#include "Cone.hpp"

#include "ConicalUtils.hpp"

namespace Slic3r {
Cone::Cone(const Vec3d &center, double radius, double angle) : center_(center), radius_(radius), angle_(angle) { calculateHeight(); }

Cone::Cone(const ModelObject *object, double angle)
{
    angle_ = angle;
    auto boundingBox = object->bounding_box_exact();
    center_          = boundingBox.center();
    center_[2]       = boundingBox.min.z();
    radius_          = boundingBox.size().x() / 2;
    calculateHeight();
    resizeToContainAllPoints(object->mesh().its.vertices);
}

Vec3d  Cone::getCenter() const { return center_; }
double Cone::getRadius() const { return radius_; }
double Cone::getAngle() const { return angle_; }
double Cone::getHeight() const { return height_; }

void Cone::setCenter(const Vec3d &center) { center_ = center; }
void Cone::setRadius(double radius)
{
    radius_ = radius;
    calculateHeight();
}
void Cone::setAngle(double angle)
{
    angle_ = angle;
    calculateHeight();
}

bool Cone::isPointinCone(const Vec3d &point) const
{
    double xyDistance = ConicalUtils::distanceXY(point, center_);

    double zDifference = point[2] - center_[2];

    if (zDifference < 0 || zDifference > height_) {
        return false;
    }

    double remainingHeight = height_ - zDifference;

    double radiusAtZ = (radius_ / height_) * remainingHeight;

    return xyDistance <= radiusAtZ;
}

void Cone::calculateHeight() { height_ = radius_ / std::tan(angle_); }

void Cone::resizeToContainPoint(const stl_vertex &point)
{
    double xyDistance = ConicalUtils::distanceXY(point, center_);

    double zDifference = point[2] - center_[2];

    if (zDifference < 0) {
        return;
    }

    if (zDifference > height_) {
        radius_ = ((zDifference * radius_) / height_) + xyDistance;
        calculateHeight();
        return;
    }

    double newR = (height_ * xyDistance) / (height_ - zDifference);

    if (newR > radius_) {
        radius_ = newR;
        calculateHeight();
    }
}

void Cone::resizeToContainAllPoints(std::vector<stl_vertex> points)
{
    for (const auto &point : points) {
        resizeToContainPoint(point);
    }
}


double Cone::getRadiusAtZ(double z) const
{
    z -= center_[2];

    if (z < 0 || z > height_) {
        return 0.0;
    }

    double remainingHeight = height_ - z;
    double radiusAtHeight  = (radius_ / height_) * remainingHeight;

    return radiusAtHeight;
}

bool Cone::isCollidingWith(const Cone &other) const
{
    const Cone *higherCone;
    const Cone *lowerCone;

    if (center_.z() > other.center_.z()) {
        higherCone = this;
        lowerCone  = &other;
    } else {
        higherCone = &other;
        lowerCone  = this;
    }

    double xyDistance = ConicalUtils::distanceXY(higherCone->center_, lowerCone->center_);

    double lowerConeRadiusAtHeight = lowerCone->getRadiusAtZ(higherCone->center_.z());

    return xyDistance <= (higherCone->radius_ + lowerConeRadiusAtHeight);
}

void Cone::checkForCollisions(const std::vector<Cone> &cones)
{
    for (size_t i = 0; i < cones.size(); ++i) {
        for (size_t j = i + 1; j < cones.size(); ++j) {
            if (cones[i].isCollidingWith(cones[j])) {
                throw std::runtime_error("Collision between objects Bounding Cones detected!");
            }
        }
    }
}
 
} // namespace Slic3r