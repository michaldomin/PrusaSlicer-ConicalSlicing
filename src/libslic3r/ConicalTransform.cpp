#include "ConicalTransform.hpp"

#include "CutUtils.hpp"

namespace Slic3r {
std::vector<ObjectInfo> ConicalTransform::apply_transform(const Model &model, const DynamicPrintConfig &config)
{
    meshes_backup.clear();
    _config = config;

	for (const auto &modelObject : model.objects) {
        meshes_backup.push_back({modelObject->mesh(), modelObject->name});
    }

    std::vector<ObjectInfo> new_meshes;

    for (const auto &modelObject : model.objects) {
        TriangleMesh transformed_mesh;
        if (_config.opt_bool("skip_first_layer")) {
            auto cut_meshes = cut_first_layer(modelObject);
            transformed_mesh = apply_transformation_on_one_mesh(TriangleMesh(cut_meshes.first));
            transformed_mesh.merge(TriangleMesh(cut_meshes.second));
        }
        else {
            transformed_mesh = apply_transformation_on_one_mesh(TriangleMesh((modelObject->mesh())));
        }
        new_meshes.push_back({transformed_mesh, modelObject->name});
    }


    return new_meshes;
}

const std::pair<indexed_triangle_set, indexed_triangle_set> ConicalTransform::cut_first_layer(ModelObject *object)
{
    //TODO: replace layer_height with first_layer_height
    const double first_layer_height = _config.opt_float("layer_height");
    const double obj_height = object->bounding_box_exact().size().z();
    Transform3d  cut_matrix         = Geometry::translation_transform(((-obj_height / 2) + first_layer_height) * Vec3d::UnitZ());

    ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper | ModelObjectCutAttribute::KeepLower |
                                          ModelObjectCutAttribute::PlaceOnCutUpper;

    object->translate(0., 0., -object->bounding_box_exact().min.z());

    auto cut = Cut(object, 0, cut_matrix, attributes);
    auto cut_objects = cut.perform_with_plane();

    return {copy_mesh(cut_objects[0]->mesh()), copy_mesh(cut_objects[1]->mesh())};
}


TriangleMesh ConicalTransform::apply_transformation_on_one_mesh(TriangleMesh mesh)
{
    indexed_triangle_set mesh_its = mesh.its;

    const double cone_angle_rad = _config.opt_int("non_planar_angle") * M_PI / 180.0;

     indexed_triangle_set transformed_mesh;

     std::vector<std::array<Vec3d, 3>> triangles;
     for (const auto &index : mesh_its.indices) {
        std::array<Vec3d, 3> triangle = {mesh_its.vertices[index[0]].template cast<double>(),
                                         mesh_its.vertices[index[1]].template cast<double>(),
                                         mesh_its.vertices[index[2]].template cast<double>()};
        triangles.push_back(triangle);
     }

     triangles = refinement_triangulation(triangles);

     std::vector<Vec3d> points, points_transformed;
     for (const auto &triangle : triangles) {
        points.push_back(triangle[0]);
        points.push_back(triangle[1]);
        points.push_back(triangle[2]);
     }

     transformation_kegel(points, points_transformed, cone_angle_rad, mesh.center());

     size_t index = 0;
     for (const auto &point : points_transformed) {
        transformed_mesh.vertices.push_back(stl_vertex(point[0], point[1], point[2]));
        if ((index + 1) % 3 == 0) {
            stl_triangle_vertex_indices indices(index - 2, index - 1, index);
            transformed_mesh.indices.push_back(indices);
        }
        ++index;
     }


    return TriangleMesh(transformed_mesh);
}

std::vector<std::array<Vec3d, 3>> ConicalTransform::refinement_triangulation(const std::vector<std::array<Vec3d, 3>> &triangles)
{
    std::vector<std::array<Vec3d, 3>> refined_array = triangles;

    for (int i = 0; i < _config.opt_int("refinement_iterations"); ++i) {
        std::vector<std::array<Vec3d, 3>> temp_array;
        for (const auto &triangle : refined_array) {
            auto refined_triangles = refinement_four_triangles(triangle);
            temp_array.insert(temp_array.end(), refined_triangles.begin(), refined_triangles.end());
        }
        refined_array = temp_array;
    }

    return refined_array;
}

std::vector<std::array<Vec3d, 3>> ConicalTransform::refinement_four_triangles(const std::array<Vec3d, 3> &triangle)
{
    const Vec3d point1     = triangle[0];
    const Vec3d point2     = triangle[1];
    const Vec3d point3     = triangle[2];
    const Vec3d midpoint12 = (point1 + point2) / 2.0;
    const Vec3d midpoint23 = (point2 + point3) / 2.0;
    const Vec3d midpoint31 = (point3 + point1) / 2.0;

    return {{point1, midpoint12, midpoint31},
            {point2, midpoint23, midpoint12},
            {point3, midpoint31, midpoint23},
            {midpoint12, midpoint23, midpoint31}};
}

void ConicalTransform::transformation_kegel(const std::vector<Vec3d> &points,
                          std::vector<Vec3d>       &points_transformed,
                          double                    cone_angle_rad,
                          const Vec3d              &center)
{
    const int c = _config.opt_bool("inward_cone") ? -1 : 1;

    for (const auto &point : points) {
        double x = point[0] - center[0];
        double y = point[1] - center[1];
        double z = point[2] - center[2];

        Vec3d transformed_point(x / std::cos(cone_angle_rad), y / std::cos(cone_angle_rad),
                                z + c * std::sqrt(x * x + y * y) * std::tan(cone_angle_rad));

        transformed_point[0] += center[0];
        transformed_point[1] += center[1];
        transformed_point[2] += center[2];

        points_transformed.push_back(transformed_point);
    }
}


std::string ConicalTransform::apply_back_transform(const std::string &gcode_layer, double height) const
{
    // TODO: replace layer_height with first_layer_height
    if (_config.opt_bool("skip_first_layer") && _config.opt_float("layer_height") + 0.01 > height) {
        return gcode_layer;
    }
    const double cone_angle_rad = _config.opt_int("non_planar_angle") * M_PI / 180.0;
    const bool   inward_cone    = _config.opt_bool("inward_cone");
    const Vec3d  center         = meshes_backup[0].mesh.center();

    std::regex pattern_X("X[-0-9]*[.]?[0-9]*");
    std::regex pattern_Y("Y[-0-9]*[.]?[0-9]*");
    std::regex pattern_Z("Z[-0-9]*[.]?[0-9]*");
    std::regex pattern_E("E[-0-9]*[.]?[0-9]*");
    std::regex pattern_G("^G[1] ");

    double x_old = 0, y_old = 0;
    double x_new = 0, y_new = 0;
    double z_layer  = height;
    double z_max    = 0;
    bool   update_x = false, update_y = false;
    int    c = inward_cone ? 1 : -1;

    std::vector<std::string> new_data = {};
    std::istringstream       gcode_stream(gcode_layer);
    std::string              row;

    while (std::getline(gcode_stream, row)) {
        std::smatch g_match;
        if (!std::regex_search(row, g_match, pattern_G)) {
            new_data.push_back(row);
            continue;
        }

        size_t      semicolon_pos = row.find(';');
        std::string comment       = "";
        if (semicolon_pos != std::string::npos) {
            comment = row.substr(semicolon_pos);
            row     = row.substr(0, semicolon_pos);
        }

        std::smatch x_match, y_match, z_match;
        bool        x_found = std::regex_search(row, x_match, pattern_X);
        bool        y_found = std::regex_search(row, y_match, pattern_Y);
        bool        z_found = std::regex_search(row, z_match, pattern_Z);

        if (!x_found && !y_found && !z_found) {
            new_data.push_back(row + comment);
            continue;
        }

        if (x_found) {
            x_new    = std::stod(x_match.str().substr(1)) - center.x();
            update_x = true;
        }
        if (y_found) {
            y_new    = std::stod(y_match.str().substr(1)) - center.y();
            update_y = true;
        }

        std::smatch e_match;
        bool        e_found  = std::regex_search(row, e_match, pattern_E);
        double      x_old_bt = x_old * std::cos(cone_angle_rad), x_new_bt = x_new * std::cos(cone_angle_rad);
        double      y_old_bt = y_old * std::cos(cone_angle_rad), y_new_bt = y_new * std::cos(cone_angle_rad);
        double      dist_transformed = std::sqrt(std::pow(x_new - x_old, 2) + std::pow(y_new - y_old, 2));

        int                 num_segm = static_cast<int>(dist_transformed / 0.5) + 1; // Assuming a default segment length of 0.5
        std::vector<double> x_vals(num_segm + 1), y_vals(num_segm + 1), z_vals(num_segm + 1);
        for (int i = 0; i <= num_segm; ++i) {
            x_vals[i] = x_old_bt + i * (x_new_bt - x_old_bt) / num_segm + center.x();
            y_vals[i] = y_old_bt + i * (y_new_bt - y_old_bt) / num_segm + center.y();
        }

        if (inward_cone && !e_found && (update_x || update_y)) {
            double z_start = z_layer + c * std::sqrt(x_old_bt * x_old_bt + y_old_bt * y_old_bt) * std::tan(cone_angle_rad);
            double z_end   = z_layer + c * std::sqrt(x_new_bt * x_new_bt + y_new_bt * y_new_bt) * std::tan(cone_angle_rad);
            for (int i = 0; i <= num_segm; ++i) {
                z_vals[i] = z_start + i * (z_end - z_start) / num_segm;
            }
        } else {
            for (int i = 0; i <= num_segm; ++i) {
                z_vals[i] = z_layer + c *
                                          std::sqrt((x_vals[i] - center.x()) * (x_vals[i] - center.x()) +
                                                    (y_vals[i] - center.y()) * (y_vals[i] - center.y())) *
                                          std::tan(cone_angle_rad);
            }
            if (e_found && (std::max_element(z_vals.begin(), z_vals.end()) != z_vals.end() || z_max == 0)) {
                z_max = *std::max_element(z_vals.begin(), z_vals.end());
            }
            if (!e_found && std::max_element(z_vals.begin(), z_vals.end()) != z_vals.end()) {
                for (auto &z : z_vals) {
                    z = std::min(z, z_max + 1);
                }
            }
        }

        std::vector<double> distances_transformed(num_segm, dist_transformed / num_segm);
        std::vector<double> distances_bt(num_segm);
        for (int i = 1; i <= num_segm; ++i) {
            distances_bt[i - 1] = std::sqrt(std::pow(x_vals[i] - x_vals[i - 1], 2) + std::pow(y_vals[i] - y_vals[i - 1], 2) +
                                            std::pow(z_vals[i] - z_vals[i - 1], 2));
        }

        std::string row_new = insert_Z(row, z_vals[0]);
        row_new             = replace_E(row_new, num_segm, 1, std::cos(cone_angle_rad));

        std::string replacement_rows;
        for (int j = 0; j < num_segm; ++j) {
            std::string single_row = std::regex_replace(row_new, pattern_X, "X" + std::to_string(round(x_vals[j + 1] * 1000.0) / 1000.0));
            single_row = std::regex_replace(single_row, pattern_Y, "Y" + std::to_string(round(y_vals[j + 1] * 1000.0) / 1000.0));
            single_row = std::regex_replace(single_row, pattern_Z,
                                            "Z" + std::to_string(z_vals[j + 1] < 0 ? z_layer : round(z_vals[j + 1] * 1000.0) / 1000.0));
            single_row = replace_E(single_row, distances_transformed[j], distances_bt[j], 1);
            replacement_rows += single_row + "\n";
        }

        if (update_x) {
            x_old    = x_new;
            update_x = false;
        }
        if (update_y) {
            y_old    = y_new;
            update_y = false;
        }

        new_data.push_back(replacement_rows + comment);
    }

    std::ostringstream oss;
    for (const auto &line : new_data) {
        oss << line << "\n";
    }

    return oss.str();
}

std::string ConicalTransform::insert_Z(const std::string &row, double z_value) const
{
    std::regex  pattern_X("X[-0-9]*[.]?[0-9]*");
    std::regex  pattern_Y("Y[-0-9]*[.]?[0-9]*");
    std::regex  pattern_Z("Z[-0-9]*[.]?[0-9]*");
    std::smatch match_x, match_y, match_z;

    std::string row_new = row;
    if (std::regex_search(row, match_z, pattern_Z)) {
        row_new = std::regex_replace(row, pattern_Z, " Z" + std::to_string(round(z_value * 1000.0) / 1000.0));
    } else if (std::regex_search(row, match_y, pattern_Y)) {
        row_new.insert(match_y.position() + match_y.length(), " Z" + std::to_string(round(z_value * 1000.0) / 1000.0));
    } else if (std::regex_search(row, match_x, pattern_X)) {
        row_new.insert(match_x.position() + match_x.length(), " Z" + std::to_string(round(z_value * 1000.0) / 1000.0));
    } else {
        row_new = "Z" + std::to_string(round(z_value * 1000.0) / 1000.0) + " " + row;
    }
    return row_new;
}

std::string ConicalTransform::replace_E(const std::string &row, double dist_old, double dist_new, double corr_value) const
{
    std::regex  pattern_E("E[-0-9]*[.]?[0-9]*");
    std::smatch match_e;
    std::string row_new = row;
    if (std::regex_search(row, match_e, pattern_E)) {
        double             e_val_old = std::stod(match_e.str().substr(1));
        double             e_val_new = (dist_old == 0) ? 0 : e_val_old * dist_new * corr_value / dist_old;
        std::ostringstream oss;
        oss << "E" << std::fixed << std::setprecision(5) << e_val_new;
        row_new.replace(match_e.position(), match_e.length(), oss.str());
    }
    return row_new;
}

double ConicalTransform::compute_angle_radial(double x_old, double y_old, double x_new, double y_new, bool inward_cone) const
{
    double angle = std::atan2(y_new, x_new);
    if (inward_cone) {
        angle += M_PI;
    }
    return angle;
}

std::vector<double> ConicalTransform::compute_U_values(const std::vector<double> &angle_array) const
{
    int                              angle_count = angle_array.size();
    std::vector<std::vector<double>> angle_candidates(angle_count, std::vector<double>(21));
    for (int i = 0; i < angle_count; ++i) {
        for (int k = -10; k <= 10; ++k) {
            angle_candidates[i][k + 10] = std::round((angle_array[i] + k * 2 * M_PI) * 10000.0) / 10000.0;
        }
    }

    std::vector<double> angle_insert = {angle_array[0]};
    for (int i = 1; i < angle_count; ++i) {
        double angle_prev = angle_insert.back();
        double min_diff   = std::abs(angle_candidates[i][0] - angle_prev);
        int    idx        = 0;
        for (int j = 1; j < 21; ++j) {
            double diff = std::abs(angle_candidates[i][j] - angle_prev);
            if (diff < min_diff) {
                min_diff = diff;
                idx      = j;
            }
        }
        angle_insert.push_back(angle_candidates[i][idx]);
    }

    for (auto &angle : angle_insert) {
        angle = std::round(angle * 360.0 / (2 * M_PI) * 100.0) / 100.0;
    }

    return angle_insert;
}

std::string ConicalTransform::insert_U(const std::string &row, double angle) const
{
    std::regex  pattern_Z("Z[-0-9]*[.]?[0-9]*");
    std::regex  pattern_U("U[-0-9]*[.]?[0-9]*");
    std::smatch match_z, match_u;

    std::string row_new = row;
    if (std::regex_search(row, match_u, pattern_U)) {
        row_new = std::regex_replace(row, pattern_U, "U" + std::to_string(angle));
    } else if (std::regex_search(row, match_z, pattern_Z)) {
        row_new.insert(match_z.position() + match_z.length(), " U" + std::to_string(angle));
    }

    return row_new;
}

}
