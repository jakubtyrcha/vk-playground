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

TEST_CASE( "Can find the child index", "[svo]" ) {
    SvoPool<4, BrickVoxelPosition::NodeCorner> pool;
    pool.reset(1, 1);
    Obb volume{.center = {}, .orientation = Mat3x3{1}, .half_extent = Vec3{1}};
    Svo svo{pool, volume, 1};
    {
        Vec3i begin{0};
        Vec3i end{4};
        REQUIRE(svo.get_child_index_and_refine_range({}, begin, end) == 0);
        REQUIRE(begin == Vec3i{0});
        REQUIRE(end == Vec3i{2, 2, 2});
    }
    {
        Vec3i begin{0};
        Vec3i end{4};
        REQUIRE(svo.get_child_index_and_refine_range({0, 1, 2}, begin, end) == 4);
        REQUIRE(begin == Vec3i{0, 0, 2});
        REQUIRE(end == Vec3i{2, 2, 4});
    }
    {
        Vec3i begin{4, 4, 8};
        Vec3i end{8, 8, 12};
        REQUIRE(svo.get_child_index_and_refine_range({7, 7, 11}, begin, end) == 7);
        REQUIRE(begin == Vec3i{6, 6, 10});
        REQUIRE(end == Vec3i{8, 8, 12});
    }
}

void require_approx_eq(Vec4 const& a, Vec4 const& b) {
    REQUIRE(a.x == Approx(b.x).epsilon(0.0001f));
    REQUIRE(a.y == Approx(b.y).epsilon(0.0001f));
    REQUIRE(a.z == Approx(b.z).epsilon(0.0001f));
    REQUIRE(a.w == Approx(b.w).epsilon(0.0001f));
}

TEMPLATE_TEST_CASE( "Can sample at the edges", "[svo][template]",
    (SvoPool<4, BrickVoxelPosition::NodeCorner>),
    (SvoPool<3, BrickVoxelPosition::NodeCorner>),
    (SvoPool<5, BrickVoxelPosition::NodeCenter>),
    (SvoPool<4, BrickVoxelPosition::NodeCenter>)
) {
    TestType pool;
    pool.reset(10000, 10000);

    Obb volume{ .center = {}, .orientation = Mat3x3{1}, .half_extent = Vec3{1} };
    i32 max_depth = 1;
    Svo svo{pool, volume, max_depth};

    for(int i=0; i<8; i++) {
        const Vec3 p{ i & 1, (i >> 1) & 1, (i >> 2) & 1 };
        svo.set_color_at_location(p * Vec3{2} - Vec3{1}, Vec4{p, 1});
    }
    
    svo.build_tree();

    for(int i=0; i<8; i++) {
        const Vec3 p{ i & 1, (i >> 1) & 1, (i >> 2) & 1 };
        Vec4 s = svo.sample_color_at_location(p * Vec3{2} - Vec3{1});
        require_approx_eq(s, Vec4{p, 1});

        Vec4 s0 = svo.sample_color_at_location_level(p * Vec3{2} - Vec3{1}, 0);
        if constexpr(TestType::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            require_approx_eq(s0, Vec4{p, 1} * Vec4{1.f / 8.f});
        }
        else if constexpr(TestType::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            require_approx_eq(s0, Vec4{p, 1} * Vec4{8.f / 27.f});
        }
    }
}

TEST_CASE( "Can store and sample from voxel-at-node-corner brick octree", "[svo]") {
    SvoPool<4, BrickVoxelPosition::NodeCorner> pool;
    pool.reset(10000, 10000);

    Obb volume{ .center = {}, .orientation = Mat3x3{1}, .half_extent = Vec3{1} };
    i32 max_depth = 1;
    Svo svo{pool, volume, max_depth};

    const f32 voxel_size = svo.get_voxel_world_size();
    REQUIRE(voxel_size == Approx(2.f / 6.f));
    const i32 res = svo.get_voxel_res(max_depth);
    REQUIRE(res == 7);
    for(i32 i=0; i<res; i++) {
        for(i32 j=0; j<res; j++) {
            svo.set_color_at_location(Vec3{-1 + i * voxel_size, -1 + j * voxel_size, -voxel_size}, Vec4{1, 0, 0, 1});
        }
    }
    
    SECTION( "sampling at voxel center" ) {
        {
            Vec4 sample = svo.sample_color_at_location({-1, -1, -voxel_size});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
    }
    SECTION( "interpolating non-border voxels" ) {
        {
            Vec4 sample = svo.sample_color_at_location({-1 + 0.5f * voxel_size, -1, -voxel_size});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({-1 + 0.5f * voxel_size, -1 + 0.5f * voxel_size, -voxel_size});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({-1 + 0.5f * voxel_size, -1 + 0.5f * voxel_size, -1.5f * voxel_size});
            require_approx_eq(sample, Vec4{0.5f, 0, 0, 0.5f});
        }
    }

    svo.build_tree();

    SECTION( "interpolating border voxels" ) {
        {
            Vec4 sample = svo.sample_color_at_location({-1 + 2.5f * voxel_size, -1,-voxel_size});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({-1, -1 + 2.5f * voxel_size, -voxel_size});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({-1 + 2.5f * voxel_size, -1 + 2.5f * voxel_size, -voxel_size});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({-1 + 2.5f * voxel_size, -1 + 2.5f * voxel_size, -0.5f * voxel_size});
            require_approx_eq(sample, Vec4{0.5f, 0, 0, 0.5f});
        }
    }

    SECTION("sample at downsampled level") {
        {
            Vec4 sample = svo.sample_color_at_location_level({-1, -1, -1.f + voxel_size * 2.f}, 0);
            require_approx_eq(sample, Vec4{0.5f, 0, 0, 0.5f});
        }
        {
            Vec4 sample = svo.sample_color_at_location_level({-1, -1, -1.f + voxel_size * 3.f}, 0);
            require_approx_eq(sample, Vec4{0.25f, 0, 0, 0.25f});
        }
    }
}

TEST_CASE( "Can store and sample from voxel-at-node-center brick octree", "[svo]") {
    SvoPool<4, BrickVoxelPosition::NodeCenter> pool;
    pool.reset(10000, 10000);

    Obb volume{ .center = {}, .orientation = Mat3x3{1}, .half_extent = Vec3{1} };
    i32 max_depth = 1;
    Svo svo{pool, volume, max_depth};

    const f32 voxel_size = svo.get_voxel_world_size();
    REQUIRE(voxel_size == Approx(0.5f));
    const i32 res = svo.get_voxel_res(max_depth);
    REQUIRE(res == 4);
    for(i32 i=0; i<res; i++) {
        for(i32 j=0; j<res; j++) {
            svo.set_color_at_location(Vec3{
                -1 + 0.5f * voxel_size + i * voxel_size, 
                -1 + 0.5f * voxel_size + j * voxel_size, 
                -0.5f * voxel_size}, 
                Vec4{1, 0, 0, 1}
            );
            svo.set_color_at_location(Vec3{
                -1 + 0.5f * voxel_size + i * voxel_size, 
                -1 + 0.5f * voxel_size + j * voxel_size, 
                0.5f * voxel_size}, 
                Vec4{0, 1, 0, 1}
            );
        }
    }

    Vec3 v000_sample {-1 + 0.5f * voxel_size};

    SECTION( "sampling at voxel center" ) {
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{0, 0, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
    }

    SECTION( "interpolating non-border voxels" ) {
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{0.5f, 0, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{0, 0.5f, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{0.5f, 0.5f, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
    }

    svo.build_tree();

    SECTION( "interpolating border voxels" ) {
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{1.5f, 0, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{0, 1.5f, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{0, 0, 1.5f}});
            require_approx_eq(sample, Vec4{0.5f, 0.5f, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{1.5f, 1.5f, 1}});
            require_approx_eq(sample, Vec4{1, 0, 0, 1});
        }
        {
            Vec4 sample = svo.sample_color_at_location({v000_sample + voxel_size * Vec3{1.5f, 1.5f, 1.5f}});
            require_approx_eq(sample, Vec4{0.5f, 0.5f, 0, 1.f});
        }
    }

    SECTION("sample at downsampled level") {
        Vec3 v000_sample_0 {-1 + 0.5f * voxel_size * 2};
        {
            Vec4 sample = svo.sample_color_at_location_level({}, 0);
            require_approx_eq(sample, Vec4{0.25f, 0.25f, 0, 0.5f});
        }
    }
}

TEMPLATE_TEST_CASE( "Can interpolate gradients", "[svo][template]",
(SvoPool<4, BrickVoxelPosition::NodeCorner>),
(SvoPool<3, BrickVoxelPosition::NodeCorner>),
(SvoPool<5, BrickVoxelPosition::NodeCenter>),
(SvoPool<4, BrickVoxelPosition::NodeCenter>)
) {
    TestType pool;
    pool.reset(10000, 10000);

    Obb volume{ .center = {}, .orientation = Mat3x3{1}, .half_extent = Vec3{1} };
    i32 max_depth = 2;
    Svo svo{pool, volume, max_depth};
    const f32 voxel_size = svo.get_voxel_world_size();

    const i32 res = svo.get_voxel_res(max_depth);
    for(i32 z = 0; z < res; z++)
    {
        for (i32 y = 0; y < res; y++)
        {
            for (i32 x = 0; x < res; x++)
            {
                Vec3 sample_pos = Vec3{x, y, z} * voxel_size - Vec3{1} + svo.get_first_voxel_world_offset();
                svo.set_color_at_location(sample_pos, Vec4{sample_pos, 1});
            }
        }
    }
    svo.build_tree();

    {
        const Vec3 ray_dir = glm::normalize(Vec3{1});
        f32 step = voxel_size * 0.5f;

        for (i32 depth = max_depth; depth >= 0; depth--)
        {
            step *= 2.f;
            Vec3 ray_origin{-1};
            // this can't interpolate well on the outer border for node center mode
            ray_origin += step * Vec3{glm::sqrt(2.f)};
            for (; glm::all(glm::lessThan(ray_origin, Vec3{1 - step * glm::sqrt(2.f)})); ray_origin += step * ray_dir)
            {
                Vec4 sample = svo.sample_color_at_location_level(ray_origin, depth);
                require_approx_eq(sample, Vec4{ray_origin, 1.f});
            }
        }
    }
    {
        const Vec3 ray_dir = glm::normalize(Vec3{1,0,0} - Vec3{-1, 1, -1});
        f32 step = voxel_size * 0.5f;

        for (i32 depth = max_depth; depth >= 0; depth--)
        {
            step *= 2.f;
            Vec3 ray_origin{1,0,0};
            // this can't interpolate well on the outer border for node center mode
            ray_origin += step * Vec3{glm::sqrt(2.f)};
            for (; glm::all(glm::lessThan(ray_origin, Vec3{1 - step * glm::sqrt(2.f)})) &&
            glm::all(glm::greaterThan(ray_origin, Vec3{-1 + step * glm::sqrt(2.f)}))
            ; ray_origin += step * ray_dir)
            {
                Vec4 sample = svo.sample_color_at_location_level(ray_origin, depth);
                require_approx_eq(sample, Vec4{ray_origin, 1.f});
            }
        }
    }
    {
        const Vec3 ray_dir = glm::normalize(Vec3{0,1,0} - Vec3{1, -1, -1});
        f32 step = voxel_size * 0.5f;

        for (i32 depth = max_depth; depth >= 0; depth--)
        {
            step *= 2.f;
            Vec3 ray_origin{0,1,0};
            // this can't interpolate well on the outer border for node center mode
            ray_origin += step * Vec3{glm::sqrt(2.f)};
            for (; glm::all(glm::lessThan(ray_origin, Vec3{1 - step * glm::sqrt(2.f)})) &&
            glm::all(glm::greaterThan(ray_origin, Vec3{-1 + step * glm::sqrt(2.f)}))
            ; ray_origin += step * ray_dir)
            {
                Vec4 sample = svo.sample_color_at_location_level(ray_origin, depth);
                require_approx_eq(sample, Vec4{ray_origin, 1.f});
            }
        }
    }
}
