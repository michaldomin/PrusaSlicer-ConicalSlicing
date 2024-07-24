#ifndef slic3r_ConicalTransform_hpp_
#define slic3r_ConicalTransform_hpp_

#include <iostream>

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include <cmath>
#include <regex>
#include "CutUtils.hpp"
#include "Cone.hpp"

namespace Slic3r {

struct ObjectInfo
{
    TriangleMesh mesh;
    std::string  name;
};

enum ConeType { outward, inward };

class ConicalTransform
{
public:
    ConicalTransform()
    {
        _pattern_X = "X[-0-9]*[.]?[0-9]*";
        _pattern_Y = "Y[-0-9]*[.]?[0-9]*";
        _pattern_Z = "Z[-0-9]*[.]?[0-9]*";
        _pattern_E = "E[-0-9]*[.]?[0-9]*";
        _pattern_U = "U[-0-9]*[.]?[0-9]*";
        _pattern_G = "^G[1] ";
    };

    std::vector<ObjectInfo> apply_transform(const Model &model, const DynamicPrintConfig &full_config);

    std::string apply_back_transform(const std::string &gcode_layer, double height) const;

    std::vector<ObjectInfo> get_backup() const { return meshes_backup; }

    bool is_backup_empty() const { return meshes_backup.empty(); }

    void clear_backup() { meshes_backup.clear(); }

    void reset_saved_values() const
    {
        _x_old = _center_x;
        _y_old = _center_y;
        _z_max = 0;
    }

private:
    DynamicPrintConfig      _config;
    std::vector<ObjectInfo> meshes_backup;
    double                  _cone_angle_rad;
    mutable bool          _inward_cone;
    double                  _center_x;
    double                  _center_y;
    double                  _planar_height;
    mutable double          _x_old;
    mutable double          _y_old;
    mutable double          _z_max;

    std::regex _pattern_X;
    std::regex _pattern_Y;
    std::regex _pattern_Z;
    std::regex _pattern_E;
    std::regex _pattern_U;
    std::regex _pattern_G;

    TriangleMesh                                                apply_transformation_on_one_mesh(TriangleMesh mesh);
    const std::pair<indexed_triangle_set, indexed_triangle_set> cut_planar_bottom(ModelObject *object);

    indexed_triangle_set refinement_triangulation(indexed_triangle_set &mesh, int iterations) const;
    static int refinement_triangulation_get_middle_point(std::unordered_map<int, std::unordered_map<int, int>> &created_points_info,
                                                         std::vector<stl_vertex>                               &vertices,
                                                         int                                                    index_a,
                                                         int                                                    index_b);
    void       transformation_kegel(indexed_triangle_set &mesh, float center_x, float center_y) const;

    std::string         insert_Z(const std::string &row, double z_value) const;
    std::string         replace_E(const std::string &row, double dist_old, double dist_new, double corr_value) const;
    double              compute_angle_radial(double x_old, double y_old, double x_new, double y_new, bool inward_cone) const;
    std::vector<double> compute_U_values(const std::vector<double> &angle_array) const;
    std::string         insert_U(const std::string &row, double angle) const;

    indexed_triangle_set copy_mesh(const TriangleMesh &mesh) const { return copy_mesh(mesh.its); }

    indexed_triangle_set copy_mesh(const indexed_triangle_set &mesh) const
    {
        indexed_triangle_set copied_mesh;

        copied_mesh.indices  = mesh.indices;
        copied_mesh.vertices = mesh.vertices;

        return copied_mesh;
    }
};
}; // namespace Slic3r

#endif