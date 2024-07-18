#include "ConicalTransform.hpp"

#include "CutUtils.hpp"

namespace Slic3r {
std::vector<ObjectInfo> ConicalTransform::apply_transform(const Model &model, const DynamicPrintConfig &config)
{
	std::cout << "Applying conical transform-new" << std::endl;

    meshes_backup.clear();
    _config = config;

	for (const auto &modelObject : model.objects) {
        std::cout << modelObject->origin_translation << std::endl;
        meshes_backup.push_back({modelObject->mesh(), modelObject->name});
    }

    std::vector<ObjectInfo> new_meshes;

    for (const auto &modelObject : model.objects) {
        auto cut_meshes = cut_first_layer(modelObject);
        auto transformed_mesh = apply_transformation_on_one_mesh(TriangleMesh(cut_meshes.first));
        transformed_mesh.merge(TriangleMesh(cut_meshes.second));
        new_meshes.push_back({transformed_mesh, modelObject->name});
    }


    return new_meshes;
}

const std::pair<indexed_triangle_set, indexed_triangle_set> ConicalTransform::cut_first_layer(ModelObject *object)
{
    std::cout << "Cutting First Layer" << std::endl;
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
    indexed_triangle_set copied_mesh;

    copied_mesh.indices  = std::move(mesh.its.indices);
    copied_mesh.vertices = std::move(mesh.its.vertices);

    mesh = TriangleMesh(copied_mesh);

    const double cone_angle_rad = _config.opt_int("non_planar_angle") * M_PI / 180.0;

     indexed_triangle_set transformed_mesh;

     std::vector<std::array<Vec3d, 3>> triangles;
     for (const auto &index : copied_mesh.indices) {
        std::array<Vec3d, 3> triangle = {copied_mesh.vertices[index[0]].template cast<double>(),
                                         copied_mesh.vertices[index[1]].template cast<double>(),
                                         copied_mesh.vertices[index[2]].template cast<double>()};
        triangles.push_back(triangle);
     }

     //triangles = cut_first_layer(triangles);
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


}
