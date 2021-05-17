#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/hash.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>

using Vec3 = glm::vec3;
using Vec4 = glm::vec4;
using Mat3 = glm::mat3;

using Vec3i = glm::ivec3;
using Vec4i = glm::ivec4;

using i32 = int32_t;
using i64 = int64_t;

using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;

struct Ray {
    Vec3 origin;
    Vec3 direction;

    bool operator == (Ray const&) const = default;
};

struct Plane {
    Vec3 origin;
    Vec3 normal;
};

struct Obb {
    Vec3 center;
    Mat3 orientation;
    Vec3 half_extent;

    Vec3 get_size() const;
    Vec3 to_local_orient(Vec3 dir) const;
    Vec3 to_local(Vec3 point) const;
    Ray to_local(Ray r) const;
    Plane to_local(Plane r);
};