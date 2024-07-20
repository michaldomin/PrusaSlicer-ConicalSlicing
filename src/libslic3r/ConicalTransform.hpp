#ifndef slic3r_ConicalTransform_hpp_
#define slic3r_ConicalTransform_hpp_

#include <iostream>

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include <cmath>
#include <regex>
#include "CutUtils.hpp"

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

    void resetSavedPosition() const
    {
        _x_old = _center_x;
        _y_old = _center_y;
    }

private:
    DynamicPrintConfig _config;
    std::vector<ObjectInfo> meshes_backup;
    double _cone_angle_rad;
    double                  _center_x;
    double                  _center_y;
    double                  _planar_height;
    mutable double          _x_old;
    mutable double          _y_old;

    std::regex _pattern_X;
    std::regex _pattern_Y;
    std::regex _pattern_Z;
    std::regex _pattern_E;
    std::regex _pattern_U;
    std::regex _pattern_G;


    TriangleMesh                             apply_transformation_on_one_mesh(TriangleMesh mesh);
    const std::pair<indexed_triangle_set, indexed_triangle_set> cut_planar_bottom(ModelObject *object);
    std::vector<std::array<Vec3d, 3>>        refinement_triangulation(const std::vector<std::array<Vec3d, 3>> &triangles);
    static std::vector<std::array<Vec3d, 3>> refinement_four_triangles(const std::array<Vec3d, 3> &triangle);
    void transformation_kegel(const std::vector<Vec3d> &points, std::vector<Vec3d> &points_transformed, double center_x, double center_y);

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
};
};

#endif