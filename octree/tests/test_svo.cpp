#include <catch2/catch.hpp>

#include "svo.h"

using namespace Catch::literals;

using Mat3x3 = glm::mat3;

TEMPLATE_TEST_CASE( "Can fill brick texels and do a bilinear sample", "[svo][template]",
    (BrickPayload<4, BrickLayout::Linear>),
    (BrickPayload<8, BrickLayout::Linear>),
    (BrickPayload<3, BrickLayout::Linear>)
) {
    TestType brick0;
    brick0.init();
    brick0.set_voxel_color({1, 1, 1}, {1, 0, 0, 1});
    {
        Vec4 sample = brick0.sample_trilinear({0.5f, 0.5f, 0.5f});
        REQUIRE(sample.x == 0.125_a);
        REQUIRE(sample.y == 0_a);
        REQUIRE(sample.z == 0_a);
        REQUIRE(sample.w == 0.125_a);
    }
    {
        Vec4 sample = brick0.sample_trilinear({0.5f, 1, 1});
        REQUIRE(sample.x == 0.5_a);
        REQUIRE(sample.y == 0_a);
        REQUIRE(sample.z == 0_a);
        REQUIRE(sample.w == 0.5_a);
    }
    {
        Vec4 sample = brick0.sample_trilinear({0, 0, 0});
        REQUIRE(sample.x == 0_a);
        REQUIRE(sample.y == 0_a);
        REQUIRE(sample.z == 0_a);
        REQUIRE(sample.w == 0_a);
    }
    {
        Vec4 sample = brick0.sample_trilinear({1.5f, 1.5f, 1.5f});
        REQUIRE(sample.x == 0.125_a);
        REQUIRE(sample.y == 0_a);
        REQUIRE(sample.z == 0_a);
        REQUIRE(sample.w == 0.125_a);
    }
    {
        Vec4 sample = brick0.sample_trilinear(Vec3{TestType::Size - 1});
        REQUIRE(sample.x == 0_a);
        REQUIRE(sample.y == 0_a);
        REQUIRE(sample.z == 0_a);
        REQUIRE(sample.w == 0_a);
    }
}

TEMPLATE_TEST_CASE( "Can build SVO from pre-authored voxels", "[svo][template]",
(SvoPool<8, BrickVoxelPosition::NodeCenter>),
(SvoPool<8, BrickVoxelPosition::NodeCorner>)
) {
    TestType pool;
    pool.reset(3, 100, 1000);

    Obb volume{ .center = {}, .orientation = Mat3x3{1}, .half_extent = Vec3{1} };
    Svo svo{pool, volume};

    const f32 voxel_size = svo.get_voxel_size();

    Vec3 pos{-0.5f, -0.5f, 0.f };
    // generate a plane
    for(; pos.y < 0.5f; pos.y += voxel_size) {
        for(; pos.x < 0.5f; pos.x += voxel_size) {
            svo.set_color_at_location(pos, Vec4{0, 1, 0, 1});
            pos.x += voxel_size;
        }
        pos.x = -0.5f;
        pos.y += voxel_size;
    }
    
    svo.build_tree();
    
    // test raycast
    REQUIRE(1 == 1);
}

// test
// different swizzle, brick params (resolution & node centers), depth (0, 1, 5)

// build plane SVO
// cast a few rays, hits & misses, ray in an opposite direction
// color interpolation, volume accumulation