#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch2/catch_all.hpp>

#include "svo.h"

enum class BrickVoxelPosition {
    NodeCenter,
    NodeCorner
};

enum class BrickLayout {
    Linear,
    Morton
};

template<i32 BRICK_SIZE, BrickLayout BRICK_LAYOUT = BrickLayout::Linear> 
struct BrickPayload {
    std::array<Vec4, cube(BRICK_SIZE)> colors;

    void init() {
        colors.fill({});
    }

    void set_voxel_color(Vec3i coord, Vec4 color) {
        colors.at(
                coord.z * square(BRICK_SIZE) +
                coord.y * BRICK_SIZE +
                coord.x
            ) = color;
    }
};

template<i32 BRICK_SIZE, BrickVoxelPosition BRICK_VOXEL_POS, BrickLayout BRICK_LAYOUT>
struct SvoPool {
    using BrickType = BrickPayload<BRICK_SIZE, BRICK_LAYOUT>;

    std::vector<OctreeNode> nodes_pool;
    std::vector<BrickType> brick_pool;

    void reset(i32 max_depth, i32 reserve_nodes, i32 reserve_bricks);
};

template<typename Pool>
struct Svo {
    Pool & pool;

};

SvoPool<8, BrickVoxelPosition::NodeCenter>;
SvoPool<8, BrickVoxelPosition::NodeCorner>;

// smallest
SvoPool<3, BrickVoxelPosition::NodeCenter>;
SvoPool<2, BrickVoxelPosition::NodeCorner>;

// test brick sampling: linear and swizzle

TEST_CASE( "Can build SVO from pre-authored voxels", "[svo]" ) {
    SvoPool<8, BrickVoxelPosition::NodeCenter> pool;
    //pool.reset(5, 100, 1000);

    Svo svo{pool, Obb{}};
    //svo.set_voxel_color();
    //svo.set_voxel_color();
    //svo.build_octree();
    //std::optional<Hit> hit = svo.cast_ray(ray);
    //REQUIRE(hit);
}

// test
// different swizzle, brick params (resolution & node centers), depth (0, 1, 5)

// build plane SVO
// cast a few rays, hits & misses, ray in an opposite direction
// color interpolation, volume accumulation