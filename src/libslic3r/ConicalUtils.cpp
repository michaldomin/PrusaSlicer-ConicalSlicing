#include "ConicalUtils.hpp"

namespace Slic3r {

Eigen::Vector3f ConicalUtils::interpolatePoint(const Eigen::Vector3f &pointA, const Eigen::Vector3f &pointB)
{
    return Eigen::Vector3f((pointA[0] + pointB[0]) / 2.0f, (pointA[1] + pointB[1]) / 2.0f, (pointA[2] + pointB[2]) / 2.0f);
}

double ConicalUtils::distanceXY(const Vec3d &point1, const Vec3d &point2)
{
    double dx = point2[0] - point1[0];
    double dy = point2[1] - point1[1];

    return std::sqrt(dx * dx + dy * dy);
}

double ConicalUtils::distanceXY(const stl_vertex &point1, const Vec3d &point2)
{
    double dx = point2[0] - point1[0];
    double dy = point2[1] - point1[1];

    return std::sqrt(dx * dx + dy * dy);
}

} // namespace Slic3r
