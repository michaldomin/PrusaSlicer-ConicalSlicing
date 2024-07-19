#ifndef slic3r_ConicalTransform_hpp_
#define slic3r_ConicalTransform_hpp_

#include <iostream>

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include <cmath>

namespace Slic3r {

struct ObjectInfo
{
    TriangleMesh mesh;
    std::string name;
};

enum ConeType{ outward, inward};

class ConicalTransform
{
public:
    ConicalTransform() = default;

    std::vector<ObjectInfo> apply_transform(const Model &model, const DynamicPrintConfig &full_config);

    std::string apply_back_transform(const std::string &gcode_layer, double height) const;

    std::vector<ObjectInfo> get_backup() const { return meshes_backup; }

    bool is_backup_empty() const { return meshes_backup.empty(); }

    void clear_backup() { meshes_backup.clear(); }

    void test() const { std::cout << "Meshes size test: " << meshes_backup.size() << std::endl; }

private:
    DynamicPrintConfig _config;
    std::vector<ObjectInfo> meshes_backup;


    TriangleMesh                             apply_transformation_on_one_mesh(TriangleMesh mesh);
    const std::pair<indexed_triangle_set, indexed_triangle_set> cut_first_layer(ModelObject *object);
    std::vector<std::array<Vec3d, 3>>        refinement_triangulation(const std::vector<std::array<Vec3d, 3>> &triangles);
    static std::vector<std::array<Vec3d, 3>> refinement_four_triangles(const std::array<Vec3d, 3> &triangle);
    void                                     transformation_kegel(const std::vector<Vec3d> &points,
                                                                  std::vector<Vec3d>       &points_transformed,
                                                                  double                    cone_angle_rad,
                                                                  const Vec3d              &center);

    std::string         insert_Z(const std::string &row, double z_value) const;
    std::string         replace_E(const std::string &row, double dist_old, double dist_new, double corr_value) const;
    double              compute_angle_radial(double x_old, double y_old, double x_new, double y_new, bool inward_cone) const;
    std::vector<double> compute_U_values(const std::vector<double> &angle_array) const;
    std::string         insert_U(const std::string &row, double angle) const;

    indexed_triangle_set copy_mesh(const TriangleMesh &mesh)
    {
        return copy_mesh(mesh.its);
    }

    indexed_triangle_set copy_mesh(const indexed_triangle_set &mesh)
    {
        indexed_triangle_set copied_mesh;

        copied_mesh.indices  = mesh.indices;
        copied_mesh.vertices = mesh.vertices;

        return copied_mesh;
    }

    Vec3d interpolate(const Vec3d &p1, const Vec3d &p2, double height)
    {
        double t = (height - p1.z()) / (p2.z() - p1.z());
        return p1 + t * (p2 - p1);
    }

    void create_cone(Eigen::MatrixXd &V, Eigen::MatrixXi &F, double radius, double height, int slices) const;

    indexed_triangle_set cut_cone_from_mesh(const indexed_triangle_set &mesh,
                                                              double                      cone_radius,
                                                              double                      cone_height,
                                                              int                         cone_slices) const;
};
};

#endif