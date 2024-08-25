#ifndef slic3r_Cone_hpp_
#define slic3r_Cone_hpp_

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include "libslic3r/Model.hpp"

namespace Slic3r {

class Cone {
public:
    Cone(const Vec3d& center, double radius, double angle);
    Cone(const ModelObject *object, double angle);

    Vec3d getCenter() const;
    double getRadius() const;
    double getAngle() const;
    double getHeight() const;

    void setCenter(const Vec3d& center);
    void setRadius(double radius);
    void setAngle(double angle);

    bool isPointinCone(const Vec3d& point) const;

    void resizeToContainPoint(const stl_vertex &point);
    void resizeToContainAllPoints(const std::vector<stl_vertex> points);

    bool isCollidingWith(const Cone& other) const;

    double getRadiusAtZ(double z) const;

    static void checkForCollisions(const std::vector<Cone> &cones);

private:
    Vec3d center_;
    double radius_;
    double angle_;
    double height_;

    void calculateHeight();

    Vec3d getTopCenter() const;

    bool areBoundingCylindersColliding(const Cone& other) const;

    bool isDistanceBetweenAxesLessThanSumOfRadii(const Cone& other, double z) const;

    bool areCirclesColliding(const Vec3d& center1, double radius1, const Vec3d& center2, double radius2) const;
};
}

#endif
