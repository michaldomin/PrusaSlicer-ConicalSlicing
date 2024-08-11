#ifndef SLIC3R_CONICAL_TRANSFORM_HPP
#define SLIC3R_CONICAL_TRANSFORM_HPP

#include <iostream>
#include <vector>
#include <string>
#include <regex>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "CutUtils.hpp"
#include "Cone.hpp"

namespace Slic3r {

struct ObjectInfo {
    TriangleMesh mesh;
    std::string  name;
};

class ConicalTransform {
public:
    ConicalTransform();

    std::vector<ObjectInfo> applyTransform(const Model &model, const DynamicPrintConfig &config);
    std::string applyBackTransform(const std::string &gcodeLayer, double height) const;

    std::vector<ObjectInfo> getBackup() const { return _meshesBackup; }
    bool isBackupEmpty() const { return _meshesBackup.empty(); }
    void clearBackup() { _meshesBackup.clear(); }
    void resetSavedValues() const;

private:
    DynamicPrintConfig _config;
    std::vector<ObjectInfo> _meshesBackup;
    float _coneAngleRad;
    mutable bool _inwardCone;
    float _centerX;
    float _centerY;
    float _planarHeight;
    mutable float _xOld;
    mutable float _yOld;
    mutable double _zMax;

    std::regex _patternX;
    std::regex _patternY;
    std::regex _patternZ;
    std::regex _patternE;
    std::regex _patternU;
    std::regex _patternG;

    // Forward Transformation
    TriangleMesh applyTransformationOnOneMesh(const TriangleMesh& mesh);
    std::pair<indexed_triangle_set, indexed_triangle_set> cutPlanarBottom(ModelObject* object);
    indexed_triangle_set refineTriangulation(indexed_triangle_set& mesh, int iterations) const;
    int getMiddlePoint(std::unordered_map<int, std::unordered_map<int, int>>& createdPointsInfo,
                       std::vector<stl_vertex>& vertices,
                       int indexA,
                       int indexB) const;
    void applyConicalTransformation(indexed_triangle_set& mesh, float centerX, float centerY) const;

    // Backward Transformation
    std::string         insertZ(const std::string &row, double zValue) const;
    std::string         replaceE(const std::string &row, double distOld, double distNew, double corrValue) const;
    double              computeRadialAngle(double xOld, double yOld, double xNew, double yNew, bool inwardCone) const;
    std::vector<double> computeUValues(const std::vector<double>&angleArray) const;
    std::string         insertU(const std::string&row, double angle) const;

    // Utils
    indexed_triangle_set copyMesh(const TriangleMesh& mesh) const;
    indexed_triangle_set copyMesh(const indexed_triangle_set& mesh) const;
};

} // namespace Slic3r

#endif // SLIC3R_CONICAL_TRANSFORM_HPP
