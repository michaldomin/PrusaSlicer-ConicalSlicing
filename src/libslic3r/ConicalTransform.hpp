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

struct BackupObjectInfo
{
    TriangleMesh mesh;
    std::string  name;
};

struct ObjectInfo
{
    ModelObject object;
    Cone         boundingCone;
};

class ConicalTransform
{
public:
    ConicalTransform();

    std::vector<BackupObjectInfo> applyTransform(const Model &model, const DynamicPrintConfig &config);
    std::string                        applyBackTransform(const std::string &gcodeLayer, double height) const;

    std::vector<BackupObjectInfo> getBackup() const { return _backupObjectsInfo; }
    bool                    isObjectsInfoEmpty() const { return _backupObjectsInfo.empty(); }
    void                          clearObjectsInfo()
    {
        _backupObjectsInfo.clear();
        _objectsInfo.clear;
    }
    void                    resetSavedValues() const;

private:
    DynamicPrintConfig      _config;
    std::vector<BackupObjectInfo> _backupObjectsInfo;
    std::vector<BackupObjectInfo> _objectsInfo;

    float _coneAngleRad;
    bool  _inwardCone;
    float _planarHeight;

    bool  _useTransformationCenter;
    float _centerX;
    float _centerY;

    mutable float  _xOld;
    mutable float  _yOld;
    mutable double _zMax;

    std::regex _patternX;
    std::regex _patternY;
    std::regex _patternZ;
    std::regex _patternE;
    std::regex _patternU;
    std::regex _patternG;

    // Forward Transformation
    TriangleMesh                                          applyTransformationOnOneMesh(const TriangleMesh &mesh);
    std::pair<indexed_triangle_set, indexed_triangle_set> cutPlanarBottom(ModelObject *object);
    indexed_triangle_set                                  refineTriangulation(indexed_triangle_set &mesh, int iterations) const;
    int  getMiddlePoint(std::unordered_map<int, std::unordered_map<int, int>> &createdPointsInfo,
                        std::vector<stl_vertex>                               &vertices,
                        int                                                    indexA,
                        int                                                    indexB) const;
    void applyConicalTransformation(indexed_triangle_set &mesh, float centerX, float centerY) const;

    // Backward Transformation
    std::string         insertZ(const std::string &row, double zValue) const;
    std::string         replaceE(const std::string &row, double distOld, double distNew, double corrValue) const;
    double              computeRadialAngle(double xOld, double yOld, double xNew, double yNew, bool inwardCone) const;
    std::vector<double> computeUValues(const std::vector<double> &angleArray) const;
    std::string         insertU(const std::string &row, double angle) const;

    // Utils
    indexed_triangle_set copyMesh(const TriangleMesh &mesh) const;
    indexed_triangle_set copyMesh(const indexed_triangle_set &mesh) const;
};

} // namespace Slic3r

#endif // SLIC3R_CONICAL_TRANSFORM_HPP
