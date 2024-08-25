#include "ConicalTransform.hpp"

namespace Slic3r {

 std::vector<BackupObjectInfo> ConicalTransform::applyTransform(const Model &model, const DynamicPrintConfig &config)
 {
     _backupObjectsInfo.clear();
     _objectsInfo.clear();
     _config = config;
     _coneAngleRad = _config.opt_int("non_planar_angle") * M_PI / 180.0;

     for (const auto &modelObject : model.objects) {
         const auto mesh = modelObject->mesh();
         _objectsInfo.push_back({modelObject, Cone(modelObject, _coneAngleRad)});
         std::cout << _backupObjectsInfo.back().boundingCone.getHeight() << std::endl;
         std::cout << _backupObjectsInfo.back().boundingCone.getRadius() << std::endl;
     } 

     _planarHeight = _config.opt_float("planar_height");
     _useTransformationCenter = _config.opt_bool("use_own_transformation_center");

     if (_useTransformationCenter) {
         _centerX = _config.opt_float("transformation_center_x");
         _centerY = _config.opt_float("transformation_center_y");
     }
     resetSavedValues();

     std::vector<BackupObjectInfo> newMeshes;

     for (const auto &modelObject : model.objects) {
         TriangleMesh transformedMesh;
         if (_planarHeight > 0.001) {
             auto cutMeshes = cutPlanarBottom(modelObject);
             transformedMesh = applyTransformationOnOneMesh(TriangleMesh(cutMeshes.first));
             transformedMesh.merge(TriangleMesh(cutMeshes.second));
         } else {
             transformedMesh = applyTransformationOnOneMesh(TriangleMesh((modelObject->mesh())));
         }
         newMeshes.push_back({transformedMesh, modelObject->name});
     }

     return newMeshes;
 }

 std::pair<indexed_triangle_set, indexed_triangle_set> ConicalTransform::cutPlanarBottom(ModelObject *object)
 {
     const double objHeight = object->bounding_box_exact().size().z();
     Transform3d cutMatrix = Geometry::translation_transform(((-objHeight / 2) + _planarHeight) * Vec3d::UnitZ());

     ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper | ModelObjectCutAttribute::KeepLower |
                                           ModelObjectCutAttribute::PlaceOnCutUpper;

     object->translate(0., 0., -object->bounding_box_exact().min.z());

     auto cut = Cut(object, 0, cutMatrix, attributes);
     auto cutObjects = cut.perform_with_plane();

     return {copyMesh(cutObjects[0]->mesh()), copyMesh(cutObjects[1]->mesh())};
 }

 TriangleMesh ConicalTransform::applyTransformationOnOneMesh(const TriangleMesh &mesh)
 {
     indexed_triangle_set meshITS = mesh.its;

     auto refinedMesh = refineTriangulation(meshITS, _config.opt_int("refinement_iterations"));
     applyConicalTransformation(refinedMesh, _centerX, _centerY);

     return TriangleMesh(refinedMesh);
 }

 indexed_triangle_set ConicalTransform::refineTriangulation(indexed_triangle_set &mesh, int iterations) const
 {
     if (iterations <= 0) {
         return copyMesh(mesh);
     }

     std::unordered_map<int, std::unordered_map<int, int>> createdPointsInfo;
     std::vector<stl_vertex> vertices = std::move(mesh.vertices);

     std::vector<stl_triangle_vertex_indices> indices;

     for (const auto &index : mesh.indices) {
         int indexA = index[0];
         int indexB = index[1];
         int indexC = index[2];

         int indexAB = getMiddlePoint(createdPointsInfo, vertices, indexA, indexB);
         int indexBC = getMiddlePoint(createdPointsInfo, vertices, indexB, indexC);
         int indexCA = getMiddlePoint(createdPointsInfo, vertices, indexC, indexA);

         indices.emplace_back(indexA, indexAB, indexCA);
         indices.emplace_back(indexAB, indexB, indexBC);
         indices.emplace_back(indexBC, indexC, indexCA);
         indices.emplace_back(indexAB, indexBC, indexCA);
     }

     indexed_triangle_set newMesh;
     newMesh.vertices = vertices;
     newMesh.indices = indices;

     return refineTriangulation(newMesh, iterations - 1);
 }

 int ConicalTransform::getMiddlePoint(std::unordered_map<int, std::unordered_map<int, int>>& createdPointsInfo,
                                      std::vector<stl_vertex>& vertices,
                                      int indexA,
                                      int indexB) const
 {
     if (createdPointsInfo.find(indexA) != createdPointsInfo.end() &&
         createdPointsInfo[indexA].find(indexB) != createdPointsInfo[indexA].end()) {
         return createdPointsInfo[indexA][indexB];
     }

     if (createdPointsInfo.find(indexB) != createdPointsInfo.end() &&
         createdPointsInfo[indexB].find(indexA) != createdPointsInfo[indexB].end()) {
         return createdPointsInfo[indexB][indexA];
     }

     const auto pointA = vertices[indexA];
     const auto pointB = vertices[indexB];

     vertices.emplace_back((pointA[0] + pointB[0]) / 2.0, (pointA[1] + pointB[1]) / 2.0, (pointA[2] + pointB[2]) / 2.0);
     const int middleIndex = vertices.size() - 1;
     createdPointsInfo[indexA][indexB] = middleIndex;
     return middleIndex;
 }

 void ConicalTransform::applyConicalTransformation(indexed_triangle_set &mesh, float centerX, float centerY) const
 {
     const float c = _inwardCone ? -1 : 1;
     const float coneAngleRad = _coneAngleRad;

     for (auto &vertex : mesh.vertices) {
         const float x = vertex[0] - centerX;
         const float y = vertex[1] - centerY;

         vertex[0] = (x / std::cos(coneAngleRad)) + centerX;
         vertex[1] = (y / std::cos(coneAngleRad)) + centerY;
         vertex[2] += c * std::sqrt(x * x + y * y) * std::tan(coneAngleRad);
     }
 }

} // namespace Slic3r
