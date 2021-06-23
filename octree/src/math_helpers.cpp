#include "math_helpers.h" 

bool Ray::operator == (Ray const& other) const {
    return origin == other.origin && direction == other.direction;
}

Vec3 Obb::get_size() const
{
    return half_extent * 2.f;
}

Vec3 Obb::to_local_orient(Vec3 dir) const
{
    return glm::transpose(orientation) * dir;
}

Vec3 Obb::to_local(Vec3 point) const
{
    return glm::transpose(orientation) * (point - center);
}

Ray Obb::to_local(Ray r) const
{
    return Ray{.origin = to_local(r.origin), .direction = to_local_orient(r.direction)};
}

Plane Obb::to_local(Plane r)
{
    return Plane{.origin = to_local(r.origin), .normal = to_local_orient(r.normal)};
}