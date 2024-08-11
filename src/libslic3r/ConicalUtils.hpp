#ifndef SLIC3R_CONICAL_UTILS_HPP
#define SLIC3R_CONICAL_UTILS_HPP

#include <unordered_map>
#include <vector>
#include "libslic3r/Model.hpp"
#include "Eigen/Dense"

namespace Slic3r {

class ConicalUtils {
public:
    static Eigen::Vector3f interpolatePoint(const Eigen::Vector3f& pointA, const Eigen::Vector3f& pointB);
};

} // namespace Slic3r

#endif // SLIC3R_CONICAL_UTILS_HPP
