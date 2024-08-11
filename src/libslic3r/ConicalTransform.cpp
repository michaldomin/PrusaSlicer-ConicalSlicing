#include "ConicalTransform.hpp"

namespace Slic3r {

ConicalTransform::ConicalTransform()
{
    _patternX = std::regex("X[-0-9]*[.]?[0-9]*");
    _patternY = std::regex("Y[-0-9]*[.]?[0-9]*");
    _patternZ = std::regex("Z[-0-9]*[.]?[0-9]*");
    _patternE = std::regex("E[-0-9]*[.]?[0-9]*");
    _patternU = std::regex("U[-0-9]*[.]?[0-9]*");
    _patternG = std::regex("^G[1] ");
}

 void ConicalTransform::resetSavedValues() const
 {
     _xOld = _centerX;
     _yOld = _centerY;
     _zMax = 0;
 }

 indexed_triangle_set ConicalTransform::copyMesh(const TriangleMesh &mesh) const
 {
     return copyMesh(mesh.its);
 }

 indexed_triangle_set ConicalTransform::copyMesh(const indexed_triangle_set &mesh) const
 {
     indexed_triangle_set copiedMesh;

     copiedMesh.indices = mesh.indices;
     copiedMesh.vertices = mesh.vertices;

     return copiedMesh;
 }

} // namespace Slic3r