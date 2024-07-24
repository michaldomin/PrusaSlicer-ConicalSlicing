#include "Cone.hpp"

namespace Slic3r {
Cone::Cone(const Vec3d &center, double radius, double angle) : center_(center), radius_(radius), angle_(angle) { calculateHeight(); }

Vec3d  Cone::getCenter() const { return center_; }
double Cone::getRadius() const { return radius_; }
double Cone::getAngle() const { return angle_; }
double Cone::getHeight() const { return height_; }

 void Cone::setCenter(const Vec3d& center) { center_ = center; }
 void Cone::setRadius(double radius) { radius_ = radius; calculateHeight(); }
 void Cone::setAngle(double angle) { angle_ = angle; calculateHeight(); }

 //bool Cone::isPointinCone(const Vec3d& point) const {
 //     Vec3d distance = (point - center_.);

 //     double distanceXY = std::sqrt(relativePoint.x() * relativePoint.x() + relativePoint.y() * relativePoint.y());

 //     if (relativePoint.z() < 0 || relativePoint.z() > height_) {
 //         return false;
 //     }

 //     double pointAngle = std::atan2(distanceXY, relativePoint.z());

 //     return std::abs(pointAngle - angle_) < 1e-6;
 // }

void Cone::calculateHeight() { height_ = radius_ / std::tan(angle_); }
} // namespace Slic3r

// void Cone::resizeToContainPoint(const Vec3d& point) {
//     Vec3d relativePoint = point - center_;

//     // Calculate the new height based on the Z coordinate of the point
//     double newHeight = relativePoint.z();

//     // Calculate the new radius based on the height and the angle
//     double newRadius = newHeight * std::tan(angle_);

//     // Update the radius and height
//     radius_ = newRadius;
//     height_ = newHeight;
// }

// // Function to check if two cones are colliding
// bool Cone::isCollidingWith(const Cone& other) const {
//     // Check bounding cylinder intersection
//     if (!areBoundingCylindersColliding(other)) {
//         return false;
//     }

//     // Check if the distance between the cone axes is less than the sum of their radii at any height
//     double maxHeight = std::max(height_, other.height_);
//     for (double z = 0; z <= maxHeight; z += maxHeight / 100) {
//         if (isDistanceBetweenAxesLessThanSumOfRadii(other, z)) {
//             return true;
//         }
//     }

//     // Check top and bottom circle intersections
//     if (areCirclesColliding(center_, radius_, other.center_, other.radius_) ||
//         areCirclesColliding(getTopCenter(), 0, other.getTopCenter(), 0)) {
//         return true;
//     }

//     return false;
// }

// // Function to get the radius at a specific height
// double Cone::getRadiusAtHeight(double height) const {
//     // Ensure the height is within the bounds of the cone
//     if (height < 0 || height > this->height_) {
//         throw std::out_of_range("Height is out of the bounds of the cone.");
//     }
//     return height * std::tan(angle_);
// }

// // Calculate height based on radius and angle
// void Cone::calculateHeight() {
//     height_ = radius_ / std::tan(angle_);
// }

// // Get the center of the top circle
// Vec3d Cone::getTopCenter() const {
//     return Vec3d(center_.x(), center_.y(), center_.z() + height_);
// }

// // Check if the bounding cylinders of the cones are colliding
// bool Cone::areBoundingCylindersColliding(const Cone& other) const {
//     // Check if the heights overlap
//     if (center_.z() + height_ < other.center_.z() || other.center_.z() + other.height_ < center_.z()) {
//         return false;
//     }

//     // Check if the bases overlap
//     double distance = std::sqrt(std::pow(center_.x() - other.center_.x(), 2) +
//                                 std::pow(center_.y() - other.center_.y(), 2));
//     return distance <= (radius_ + other.radius_);
// }

// // Check if the distance between the cone axes is less than the sum of their radii at a given height
// bool Cone::isDistanceBetweenAxesLessThanSumOfRadii(const Cone& other, double z) const {
//     // Calculate the radii at the given height
//     double radiusAtHeight1 = z * std::tan(angle_);
//     double radiusAtHeight2 = z * std::tan(other.angle_);

//     // Calculate the distance between the cone axes at the given height
//     Vec3d point1(center_.x(), center_.y(), center_.z() + z);
//     Vec3d point2(other.center_.x(), other.center_.y(), other.center_.z() + z);
//     double distance = (point1 - point2).norm();

//     return distance <= (radiusAtHeight1 + radiusAtHeight2);
// }

// // Check if two circles are colliding
// bool Cone::areCirclesColliding(const Vec3d& center1, double radius1, const Vec3d& center2, double radius2) const {
//     double distance = std::sqrt(std::pow(center1.x() - center2.x(), 2) +
//                                 std::pow(center1.y() - center2.y(), 2) +
//                                 std::pow(center1.z() - center2.z(), 2));
//     return distance <= (radius1 + radius2);
// }
