#include <VkBootstrap.h>
#include <fmt/core.h>

#include "window.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/intersect.hpp>
#include <glm/gtx/hash.hpp>
#include <array>

using Vec3 = glm::vec3;
using Vec4 = glm::vec4;
using Mat3 = glm::mat3;

using Vec3i = glm::ivec3;

using i32 = int32_t;
using i64 = int64_t;

using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;

struct SparseOctreeBrickConfig {
    enum BrickVoxelPosition {
        BrickVoxelNodeCenter,
        BrickVoxelNodeCorner
    };

    static constexpr BrickVoxelPosition brick_voxel_position = BrickVoxelNodeCenter;
    static constexpr int brick_size = 8;
};

struct OctreeNode {
    u32 max_subdivision : 1; //
    u32 data_type_flag : 1; // node/constant
    u32 address : 30;

    union {
        struct {
            u32 r : 8;
            u32 g : 8;
            u32 b : 8;
            u32 a : 8;
        };
        struct {
            u32 x : 10;
            u32 y : 10;
            u32 z : 10;
            u32 _ : 2;
        };
    };
};

struct SvoPool {
    std::vector<OctreeNode> nodes_pool;
    std::vector<Vec3> brick_pool;
};

struct Ray {
    Vec3 origin;
    Vec3 direction;
};

struct Plane {
    Vec3 origin;
    Vec3 normal;
};

struct Obb {
    Vec3 center;
    Mat3 orientation;
    Vec3 half_extent;

    Vec3 get_size() const {
        return half_extent * 2.f;
    }

    // todo: test
    Vec3 to_local_orient(Vec3 dir) const {
        return orientation * dir;
    }

    Vec3 to_local(Vec3 point) const {
        return center + orientation * point;
    }

    Ray transform_to_local(Ray r) const {
        return Ray{ .origin = to_local(r.origin), .direction = to_local_orient(r.direction) };
    }

    Plane transform_to_local(Plane r) {
        return Plane{ .origin = to_local(r.origin), .normal = to_local_orient(r.normal) };
    }
};

template<typename T>
constexpr T square(T x) {
    return x * x;
}

template<typename T>
constexpr T cube(T x) {
    return x * x * x;
}

struct BrickPayload {
    std::array<Vec4, cube(SparseOctreeBrickConfig::brick_size)> colors;

    void init() {
        colors.fill({});
    }
    void set_voxel_color(Vec3i coord, Vec4 color) {
        colors.at(
                coord.z * square(SparseOctreeBrickConfig::brick_size) +
                coord.y * SparseOctreeBrickConfig::brick_size +
                coord.x
            ) = color;
    }
};

void generate_plane(SvoPool & pool, Plane plane, Obb cube, i32 max_octree_depth, Vec4 color) {
    std::unordered_map<Vec3i, BrickPayload> bricks;

    const Plane local_plane = cube.transform_to_local(plane);
    const f32 cube_side_len = cube.get_size().x;
    const i32 bricks_num_max_depth = (1 << max_octree_depth);

    f32 voxel_size;
    i32 voxel_res;
    if( SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCenter ) {
        voxel_res = bricks_num_max_depth * (SparseOctreeBrickConfig::brick_size - 2);
        // *-|-*--*-|-*
        voxel_size = cube_side_len / voxel_res;
    }
    else if( SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCorner ) {
        voxel_res = bricks_num_max_depth * (SparseOctreeBrickConfig::brick_size - 1) + 1;
        // |*--*--*|
        voxel_size = cube_side_len / (voxel_res - 1);
    }
    
    Ray ray_setup{ .origin = {}, .direction = {1, 0, 0} };
    Vec3 ray_offset_dim0{0, 1, 0};
    Vec3 ray_offset_dim1{0, 0, 1};

    if(glm::dot(ray_setup.direction, local_plane.normal) == 0) {
        ray_setup.direction = {0, 1, 0};
        ray_offset_dim0 = {1, 0, 0};
    }

    if(SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCenter) {
        ray_setup.origin += (ray_offset_dim0 + ray_offset_dim1) * voxel_size * 0.5f;
    }

    for(i32 y=0; y<voxel_res; y++) {
        for(i32 x=0; x<voxel_res; x++) {
            Ray ray {.origin = ray_setup.origin + ray_offset_dim0 * (f32)x + ray_offset_dim1 * (f32)y, .direction = ray_setup.direction}; 
            f32 t = -1.f;
            if(glm::intersectRayPlane(ray.origin, ray.direction, local_plane.origin, local_plane.normal, t)) {
                Vec3 hit = ray.origin + ray.direction * t;
                Vec3i voxel_coord = (hit - ray_setup.origin) / voxel_size;
                Vec3i brick_id;
                Vec3i in_brick_coord;
                if constexpr(SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCenter) {
                    brick_id = voxel_coord / (SparseOctreeBrickConfig::brick_size - 2);
                    in_brick_coord = voxel_coord - brick_id * (SparseOctreeBrickConfig::brick_size - 2);
                }
                else if constexpr(SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCorner) {
                    brick_id = voxel_coord / (SparseOctreeBrickConfig::brick_size - 1);
                    in_brick_coord = voxel_coord - brick_id * (SparseOctreeBrickConfig::brick_size - 1);
                }

                auto brick_iter = bricks.find(brick_id);
                if(brick_iter == bricks.end()) {
                    brick_iter = bricks.insert({brick_id, BrickPayload{}}).first;
                    brick_iter->second.init();
                }
                BrickPayload& brick = brick_iter->second;
                brick.set_voxel_color(in_brick_coord, color);
            }
        }
    }

    // build 
    // traverse the bricks bottom-top to build the nodes structure
}

// test
// different swizzle, brick params (resolution & node centers), depth (0, 1, 5)

// build plane SVO
// cast a few rays, hits & misses, ray in an opposite direction
// color interpolation, volume accumulation

int main() {
    auto inst_ret = vkb::InstanceBuilder{}.request_validation_layers().use_default_debug_messenger().build();
    if(!inst_ret) {
        fmt::print("Failed to create vulkan instance");
        return -1;
    }
    vkb::Instance vkb_inst = inst_ret.value();

    auto glfw_wnd = create_window_glfw();
    auto surface = create_surface_glfw(vkb_inst.instance, glfw_wnd);

    vkb::PhysicalDeviceSelector selector{ vkb_inst };
    auto phys_ret = selector.set_surface (surface)
                        .require_dedicated_transfer_queue ()
                        .select ();
    if (!phys_ret) {
        fmt::print("Failed to select Vulkan Physical Device: {}", phys_ret.error().message());
        return -1;
    }

    vkb::DeviceBuilder device_builder{ phys_ret.value () };
    auto dev_ret = device_builder.build ();
    if (!dev_ret) {
        fmt::print("Failed to create Vulkan device: ", dev_ret.error().message());
        return false;
    }
    vkb::Device vkb_device = dev_ret.value ();

    auto graphics_queue_ret = vkb_device.get_queue (vkb::QueueType::graphics);
    if (!graphics_queue_ret) {
        fmt::print("Failed to get graphics queue: ", graphics_queue_ret.error().message());
        return false;
    }
    VkQueue graphics_queue = graphics_queue_ret.value ();

    while (!glfwWindowShouldClose(glfw_wnd)) {
        glfwPollEvents();
    }

    return 0;
}