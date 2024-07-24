#include "ConicalTransform.hpp"

namespace Slic3r {
std::vector<ObjectInfo> ConicalTransform::apply_transform(const Model &model, const DynamicPrintConfig &config)
{
    meshes_backup.clear();
    _config = config;
    _cone_angle_rad = _config.opt_int("non_planar_angle") * M_PI / 180.0;

    for (const auto &modelObject : model.objects) {
        const auto mesh = modelObject->mesh();
        std::cout<< "Mesh center: " << mesh.center() << std::endl;
        std::cout << "Mesh min: " << mesh.bounding_box().min[2] << std::endl;

        auto cone = new Cone(mesh.center(), 1, _cone_angle_rad);
        std::cout<< "Cone height: " << cone->getHeight() << std::endl;
        meshes_backup.push_back({mesh, modelObject->name});
    }

    _planar_height  = _config.opt_float("planar_height");

    if (_config.opt_bool("use_own_transformation_center")) {
        _center_x = _config.opt_float("transformation_center_x");
        _center_y = _config.opt_float("transformation_center_y");
    } else {
        _center_x = meshes_backup[0].mesh.center().x();
        _center_y = meshes_backup[0].mesh.center().y();
    }
    reset_saved_values();

    std::vector<ObjectInfo> new_meshes;

    for (const auto &modelObject : model.objects) {
        TriangleMesh transformed_mesh;
        if (_planar_height > 0.001) {
            auto cut_meshes  = cut_planar_bottom(modelObject);
            transformed_mesh = apply_transformation_on_one_mesh(TriangleMesh(cut_meshes.first));
            transformed_mesh.merge(TriangleMesh(cut_meshes.second));
        } else {
            transformed_mesh = apply_transformation_on_one_mesh(TriangleMesh((modelObject->mesh())));
        }
        new_meshes.push_back({transformed_mesh, modelObject->name});
    }

    return new_meshes;
}

const std::pair<indexed_triangle_set, indexed_triangle_set> ConicalTransform::cut_planar_bottom(ModelObject *object)
{
    const double obj_height = object->bounding_box_exact().size().z();
    Transform3d  cut_matrix = Geometry::translation_transform(((-obj_height / 2) + _planar_height) * Vec3d::UnitZ());

    ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper | ModelObjectCutAttribute::KeepLower |
                                          ModelObjectCutAttribute::PlaceOnCutUpper;

    object->translate(0., 0., -object->bounding_box_exact().min.z());

    auto cut         = Cut(object, 0, cut_matrix, attributes);
    auto cut_objects = cut.perform_with_plane();

    return {copy_mesh(cut_objects[0]->mesh()), copy_mesh(cut_objects[1]->mesh())};
}

TriangleMesh ConicalTransform::apply_transformation_on_one_mesh(TriangleMesh mesh)
{
    indexed_triangle_set mesh_its = mesh.its;

    auto refined_mesh = refinement_triangulation(mesh_its, _config.opt_int("refinement_iterations"));
    transformation_kegel(refined_mesh, _center_x, _center_y);

    return TriangleMesh(refined_mesh);
}

indexed_triangle_set ConicalTransform::refinement_triangulation(indexed_triangle_set &mesh, int iterations) const
{
    if (iterations <= 0) {
        return copy_mesh(mesh);
    }

    std::unordered_map<int, std::unordered_map<int, int>> created_points_info;
    std::vector<stl_vertex>                               vertices = std::move(mesh.vertices);

    std::vector<stl_triangle_vertex_indices> indices;

    for (const auto &index : mesh.indices) {
        int index_a = index[0];
        int index_b = index[1];
        int index_c = index[2];

        int index_ab = refinement_triangulation_get_middle_point(created_points_info, vertices, index_a, index_b);
        int index_bc = refinement_triangulation_get_middle_point(created_points_info, vertices, index_b, index_c);
        int index_ca = refinement_triangulation_get_middle_point(created_points_info, vertices, index_c, index_a);

        indices.emplace_back(index_a, index_ab, index_ca);
        indices.emplace_back(index_ab, index_b, index_bc);
        indices.emplace_back(index_bc, index_c, index_ca);
        indices.emplace_back(index_ab, index_bc, index_ca);
    }

    indexed_triangle_set new_mesh;
    new_mesh.vertices = vertices;
    new_mesh.indices  = indices;

    return refinement_triangulation(new_mesh, iterations - 1);
};

int ConicalTransform::refinement_triangulation_get_middle_point(std::unordered_map<int, std::unordered_map<int, int>> &created_points_info,
                                                                std::vector<stl_vertex>                               &vertices,
                                                                int                                                    index_a,
                                                                int                                                    index_b)
{
    if (created_points_info.find(index_a) != created_points_info.end() &&
        created_points_info[index_a].find(index_b) != created_points_info[index_a].end()) {
        return created_points_info[index_a][index_b];
    }

    if (created_points_info.find(index_b) != created_points_info.end() &&
        created_points_info[index_b].find(index_a) != created_points_info[index_b].end()) {
        return created_points_info[index_b][index_a];
    }

    const auto point_a = vertices[index_a];
    const auto point_b = vertices[index_b];

    vertices.emplace_back((point_a[0] + point_b[0]) / 2.0, (point_a[1] + point_b[1]) / 2.0, (point_a[2] + point_b[2]) / 2.0);
    const int middle_index                = vertices.size() - 1;
    created_points_info[index_a][index_b] = middle_index;
    return middle_index;
};

void ConicalTransform::transformation_kegel(indexed_triangle_set &mesh, float center_x, float center_y) const
{
    const float c = _inward_cone ? -1 : 1;
    const float cone_angle_rad = _cone_angle_rad;

    for (auto &vertex : mesh.vertices) {
        const float x = vertex[0] - center_x;
        const float y = vertex[1] - center_y;

        vertex[0] = (x / std::cos(cone_angle_rad)) + center_x;
        vertex[1] = (y / std::cos(cone_angle_rad)) + center_y;
        vertex[2] += c * std::sqrt(x * x + y * y) * std::tan(cone_angle_rad);
    }
}

std::string ConicalTransform::apply_back_transform(const std::string &gcode_layer, double height) const
{
    if (_config.opt_float("planar_height") + 0.01 > height) {
        std::smatch g_match;
        std::string last_match;

        auto start = gcode_layer.cbegin();
        auto end   = gcode_layer.cend();

        while (std::regex_search(start, end, g_match, _pattern_G)) {
            last_match = g_match.str();
            start      = g_match.suffix().first;
        }

        std::smatch x_match, y_match;
        bool        x_found = std::regex_search(last_match, x_match, _pattern_X);
        bool        y_found = std::regex_search(last_match, y_match, _pattern_Y);

        if (x_found) {
            _x_old = std::stod(x_match.str().substr(1)) - _center_x;
        }
        if (y_found) {
            _y_old = std::stod(y_match.str().substr(1)) - _center_y;
        }

        return gcode_layer;
    }

    const bool inward_cone = _config.opt_bool("inward_cone");

    double x_new = 0, y_new = 0;
    double z_layer  = height;
    bool   update_x = false, update_y = false;
    int    c = inward_cone ? 1 : -1;

    std::vector<std::string> new_data = {};
    std::istringstream       gcode_stream(gcode_layer);
    std::string              row;

    while (std::getline(gcode_stream, row)) {
        std::smatch g_match;
        if (!std::regex_search(row, g_match, _pattern_G)) {
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
        bool        x_found = std::regex_search(row, x_match, _pattern_X);
        bool        y_found = std::regex_search(row, y_match, _pattern_Y);
        bool        z_found = std::regex_search(row, z_match, _pattern_Z);

        if (!x_found && !y_found && !z_found) {
            new_data.push_back(row + comment);
            continue;
        }

        if (x_found) {
            x_new    = std::stod(x_match.str().substr(1)) - _center_x;
            update_x = true;
        }
        if (y_found) {
            y_new    = std::stod(y_match.str().substr(1)) - _center_y;
            update_y = true;
        }

        std::smatch e_match;
        bool        e_found  = std::regex_search(row, e_match, _pattern_E);
        double      x_old_bt = _x_old * std::cos(_cone_angle_rad), x_new_bt = x_new * std::cos(_cone_angle_rad);
        double      y_old_bt = _y_old * std::cos(_cone_angle_rad), y_new_bt = y_new * std::cos(_cone_angle_rad);
        double      dist_transformed = std::sqrt(std::pow(x_new - _x_old, 2) + std::pow(y_new - _y_old, 2));

        int                 num_segm = static_cast<int>(dist_transformed / 0.5) + 1;
        std::vector<double> x_vals(num_segm + 1), y_vals(num_segm + 1), z_vals(num_segm + 1);
        for (int i = 0; i <= num_segm; ++i) {
            x_vals[i] = x_old_bt + i * (x_new_bt - x_old_bt) / num_segm + _center_x;
            y_vals[i] = y_old_bt + i * (y_new_bt - y_old_bt) / num_segm + _center_y;
        }

        if (inward_cone && !e_found && (update_x || update_y)) {
            double z_start = z_layer + c * std::sqrt(x_old_bt * x_old_bt + y_old_bt * y_old_bt) * std::tan(_cone_angle_rad);
            double z_end   = z_layer + c * std::sqrt(x_new_bt * x_new_bt + y_new_bt * y_new_bt) * std::tan(_cone_angle_rad);
            for (int i = 0; i <= num_segm; ++i) {
                z_vals[i] = z_start + i * (z_end - z_start) / num_segm;
            }
        } else {
            for (int i = 0; i <= num_segm; ++i) {
                z_vals[i] = z_layer + c *
                                          std::sqrt((x_vals[i] - _center_x) * (x_vals[i] - _center_x) +
                                                    (y_vals[i] - _center_y) * (y_vals[i] - _center_y)) *
                                          std::tan(_cone_angle_rad);
            }
            if (e_found) {
                _z_max = std::max(_z_max, *std::max_element(z_vals.begin(), z_vals.end()));
            } else {
                for (auto &z : z_vals) {
                    z = std::min(z, _z_max + 1);
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
        row_new             = replace_E(row_new, num_segm, 1, std::cos(_cone_angle_rad));

        std::string replacement_rows;
        for (int j = 0; j < num_segm; ++j) {
            std::string single_row = std::regex_replace(row_new, _pattern_X, "X" + std::to_string(round(x_vals[j + 1] * 1000.0) / 1000.0));
            single_row = std::regex_replace(single_row, _pattern_Y, "Y" + std::to_string(round(y_vals[j + 1] * 1000.0) / 1000.0));
            single_row = std::regex_replace(single_row, _pattern_Z,
                                            "Z" + std::to_string(z_vals[j + 1] < _planar_height ? z_layer :
                                                                                                  round(z_vals[j + 1] * 1000.0) / 1000.0));
            single_row = replace_E(single_row, distances_transformed[j], distances_bt[j], 1);
            replacement_rows += single_row + "\n";
        }

        if (update_x) {
            _x_old   = x_new;
            update_x = false;
        }
        if (update_y) {
            _y_old   = y_new;
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
    std::smatch match_x, match_y, match_z;

    std::string row_new = row;
    if (std::regex_search(row, match_z, _pattern_Z)) {
        row_new = std::regex_replace(row, _pattern_Z, " Z" + std::to_string(round(z_value * 1000.0) / 1000.0));
    } else if (std::regex_search(row, match_y, _pattern_Y)) {
        row_new.insert(match_y.position() + match_y.length(), " Z" + std::to_string(round(z_value * 1000.0) / 1000.0));
    } else if (std::regex_search(row, match_x, _pattern_X)) {
        row_new.insert(match_x.position() + match_x.length(), " Z" + std::to_string(round(z_value * 1000.0) / 1000.0));
    } else {
        row_new = "Z" + std::to_string(round(z_value * 1000.0) / 1000.0) + " " + row;
    }
    return row_new;
}

std::string ConicalTransform::replace_E(const std::string &row, double dist_old, double dist_new, double corr_value) const
{
    std::smatch match_e;
    std::string row_new = row;
    if (std::regex_search(row, match_e, _pattern_E)) {
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
    std::smatch match_z, match_u;

    std::string row_new = row;
    if (std::regex_search(row, match_u, _pattern_U)) {
        row_new = std::regex_replace(row, _pattern_U, "U" + std::to_string(angle));
    } else if (std::regex_search(row, match_z, _pattern_Z)) {
        row_new.insert(match_z.position() + match_z.length(), " U" + std::to_string(angle));
    }

    return row_new;
}

} // namespace Slic3r
