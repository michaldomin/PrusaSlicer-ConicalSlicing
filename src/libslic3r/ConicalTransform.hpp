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
    std::vector<ObjectInfo> apply_transform(const Model &model, const DynamicPrintConfig &config);

    std::vector<ObjectInfo> get_backup() { return meshes_backup; }

    bool is_backup_empty() const { return meshes_backup.empty(); }

    void clear_backup() { meshes_backup.clear(); }


    //void test();

private:
    DynamicPrintConfig _config;
    std::vector<ObjectInfo> meshes_backup;


    TriangleMesh                             apply_transformation_on_one_mesh(TriangleMesh mesh);
    std::vector<std::array<Vec3d, 3>>        refinement_triangulation(const std::vector<std::array<Vec3d, 3>> &triangles);
    static std::vector<std::array<Vec3d, 3>> refinement_four_triangles(const std::array<Vec3d, 3> &triangle);
    void                                     transformation_kegel(const std::vector<Vec3d> &points,
                                                                  std::vector<Vec3d>       &points_transformed,
                                                                  double                    cone_angle_rad,
                                                                  const Vec3d              &center);
};
};

#endif