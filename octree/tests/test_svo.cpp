#include <catch2/catch.hpp>

#include "svo.h"

using namespace Catch::literals;

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
    Obb volume{.center = {}, .orientation = Mat3{1}, .half_extent = Vec3{1}};
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

    Obb volume{ .center = {}, .orientation = Mat3{1}, .half_extent = Vec3{1} };
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
        if constexpr(TestType::SELF_BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            require_approx_eq(s0, Vec4{p, 1} * Vec4{1.f / 8.f});
        }
        else if constexpr(TestType::SELF_BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            require_approx_eq(s0, Vec4{p, 1} * Vec4{8.f / 27.f});
        }
    }
}

TEST_CASE( "Can store and sample from voxel-at-node-corner brick octree", "[svo]") {
    SvoPool<4, BrickVoxelPosition::NodeCorner> pool;
    pool.reset(10000, 10000);

    Obb volume{ .center = {}, .orientation = Mat3{1}, .half_extent = Vec3{1} };
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

    Obb volume{ .center = {}, .orientation = Mat3{1}, .half_extent = Vec3{1} };
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

    Obb volume{ .center = {}, .orientation = Mat3{1}, .half_extent = Vec3{1} };
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

TEST_CASE("Can trace ray through the brick", "[svo_trace]")
{
    SECTION("Trace for a voxel in the middle")
    {
        BrickPayload<4, BrickLayout::Linear> brick;
        brick.init();

        brick.set_voxel_color({1, 1, 1}, Vec4{1});
        {
            Vec3 origin = Vec3{0.3, 0.3, 0};
            Vec3 dir = Vec3{0, 0, 1};
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 4.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 0};
            Vec3 dir = glm::normalize(Vec3{0.000001, 0.000001, 1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 4.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 0};
            Vec3 dir = glm::normalize(Vec3{-0.000001, 0.000001, 1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 4.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 0};
            Vec3 dir = glm::normalize(Vec3{-0.000001, -0.000001, 1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 4.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 1};
            Vec3 dir = Vec3{0, 0, -1};
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 2.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 1};
            Vec3 dir = glm::normalize(Vec3{0.000001, 0.000001, -1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 2.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 1};
            Vec3 dir = glm::normalize(Vec3{-0.000001, 0.000001, -1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 2.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 1};
            Vec3 dir = glm::normalize(Vec3{0.000001, -0.000001, -1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 2.f));
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, 1};
            Vec3 dir = glm::normalize(Vec3{-0.000001, -0.000001, -1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(1.f / 2.f));
        }
        {
            Vec3 origin = Vec3{1, 0.3f, 1};
            Vec3 dir = glm::normalize(Vec3{-1, 0, -1});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(sqrtf(2 * square(0.5f))));
        }
        {
            Vec3 origin = Vec3{1, 0.3f, 1};
            Vec3 dir = glm::normalize(Vec3{-1, 0, 0});
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(!result);
        }
        {
            Vec3 origin = Vec3{0.3, 0.3, -2};
            Vec3 dir = Vec3{0, 0, 1};
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(2.f + 1.f / 4.f));
        }
    }
    SECTION("Trace for a voxel at the edge") {
        BrickPayload<4, BrickLayout::Linear> brick;
        brick.init();

        brick.set_voxel_color({0, 0, 0}, Vec4{1});
        brick.set_voxel_color({3, 3, 3}, Vec4{1});

        {
            Vec3 origin = Vec3{0.2f, 0.2f, 1};
            Vec3 dir = Vec3{0, 0, -1};
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(0.75f));
        }

        {
            Vec3 origin = Vec3{0.9f, 0.9f, 0};
            Vec3 dir = Vec3{0, 0, 1};
            Vec3 inv_dir = 1.f / dir;
            auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
            REQUIRE(result);
            REQUIRE(result->t == Approx(0.75f));
        }
    }

    SECTION("Hole-free brick tracing") {
        /* First step of tracing is moving the ray to the bounding volume of the brick.
        Sometimes the ray ends up slightly outside of the volume (despite moving by t + epsilon).
        Test that the epsilon is large enough or the method is robust.
         */
        BrickPayload<4, BrickLayout::Linear> brick;
        brick.init();

        brick.set_voxel_color({0, 0, 0}, Vec4{1});
        brick.set_voxel_color({1, 0, 0}, Vec4{1});
        brick.set_voxel_color({0, 1, 0}, Vec4{1});
        brick.set_voxel_color({1, 1, 0}, Vec4{1});

        {
            Vec3 origin = Vec3{0.f, 0.f, -1.f};
            for(int x=0; x<16; x++) {
                    for(int y=0; y<16; y++) {
                        Vec3 dir = glm::normalize(Vec3{(x + 0.5f) * 0.25f / 16.f, (y + 0.5f) * 0.25f / 16.f, 1});
                        Vec3 inv_dir = 1.f / dir;
                        auto result = Tracing::trace_brick_ray(brick, origin, dir, inv_dir);
                        REQUIRE(result);
                        REQUIRE(result->t >= 1.f);
                        REQUIRE(result->t <= sqrtf(1 + 2 * square(0.25f)));
                }
            }
        }
    }
}

TEMPLATE_TEST_CASE("Can trace ray through the SVO nodes", "[svo_trace][template]",
    (SvoPool<4, BrickVoxelPosition::NodeCorner>)
   ,(SvoPool<5, BrickVoxelPosition::NodeCorner>)
   ,(SvoPool<4, BrickVoxelPosition::NodeCenter>)
   ,(SvoPool<5, BrickVoxelPosition::NodeCenter>)
)
{
    TestType pool;
    SECTION("Trace a voxel at the edge")
    {
        pool.reset(10000, 10000);
        Obb volume{.center = Vec3{0.5f}, .orientation = Mat3{1}, .half_extent = Vec3{0.5f}};
        i32 max_depth = 5;
        Svo svo{pool, volume, max_depth};

        // fill voxel and trace a ray through the same location
        svo.set_color_at_location(Vec3{0, 0, 0}, Vec4{1, 1, 1, 1});
        svo.set_color_at_location(Vec3{1, 1, 1}, Vec4{1, 1, 1, 1});
        svo.build_tree();

        SECTION("From the edges")
        {
            auto maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{0.f}, .direction = Vec3{1, 0, 0}});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{.0001f}, .direction = Vec3{1, 0, 0}});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = Vec3{-0.0001f, 0, 0}, .direction = Vec3{1, 0, 0}});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{1.f}, .direction = Vec3{-1, 0, 0}});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{0.9999f}, .direction = Vec3{-1, 0, 0}});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = Vec3{1.0001f, 1.f, 1.f}, .direction = Vec3{-1, 0, 0}});
            REQUIRE(maybe_hit);
        }
        SECTION("From the middle")
        {
            auto maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{0.5f, 0.5f, 0.5f}, .direction = glm::normalize(Vec3{-1, -1, -1})});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{0.5f, 0.5f, 0.5f}, .direction = glm::normalize(Vec3{1, 1, 1})});
            REQUIRE(maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{0.5f, 0.5f, 0.5f}, .direction = glm::normalize(Vec3{1, 0, 0})});
            REQUIRE(!maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{0.5f, 0.5f, 0.5f}, .direction = glm::normalize(Vec3{-1, 0, 0})});
            REQUIRE(!maybe_hit);
            maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = Vec3{1, 1, 0.5}, .direction = glm::normalize(Vec3{-1, -1, -1})});
            REQUIRE(!maybe_hit);
        }
    }

    SECTION("Trace through multiple nodes")
    {
        pool.reset(10000, 10000);
        Obb volume{.center = Vec3{0.5f}, .orientation = Mat3{1}, .half_extent = Vec3{0.5f}};
        i32 max_depth = 2;
        Svo svo{pool, volume, max_depth};

        //
        i32 voxel_x = 0;
        i32 voxel_y = svo.get_voxel_res(max_depth) / 2;
        i32 voxel_z = svo.get_voxel_res(max_depth) - 1;
        const auto voxel = Vec3i{voxel_x, voxel_y, voxel_z};

        // fill voxel and trace a ray through the same location
        svo.set_color_at_leaf_node_voxel(voxel, Vec4{1, 1, 1, 1});
        svo.build_tree();

        SECTION("Trace from the voxel")
        {
            Vec3 sample_location = svo.get_leaf_node_voxel_wposition(voxel);
            auto maybe_hit = Tracing::trace_svo_local_ray_starting_within(svo, {.origin = sample_location, .direction = Vec3{1, 0, 0}});
            REQUIRE(maybe_hit);
        }
        SECTION("Trace from nearby voxels, all direction and axis combinations")
        {
            std::array<Vec3i, 3> dirs = {{{1,0,0},{0,1,0},{0,0,1}}};
            for (auto dir : dirs)
            {
                for (i32 i = 1; i <= 16; i *= 2)
                {
                    Vec3 sample_location = svo.get_leaf_node_voxel_wposition(voxel + dir * i);
                    auto maybe_hit = Tracing::trace_svo_ray(svo, {.origin = sample_location, .direction = -dir});
                    REQUIRE(maybe_hit);
                    sample_location = svo.get_leaf_node_voxel_wposition(voxel - dir * i);
                    maybe_hit = Tracing::trace_svo_ray(svo, {.origin = sample_location, .direction = dir});
                    REQUIRE(maybe_hit);
                }
            }
        }
    }

    SECTION("Trace from outside of the brick")
    {
        pool.reset(10000, 10000);

        Obb volume{.center = Vec3{0.5f}, .orientation = Mat3{1}, .half_extent = Vec3{0.5f}};
        i32 max_depth = 2;
        Svo svo{pool, volume, max_depth};

        svo.set_color_at_leaf_node_voxel({}, Vec4{1, 1, 1, 1});
        svo.build_tree();

        auto maybe_hit = Tracing::trace_svo_ray(svo, {
            .origin = {1, 0, -1}, 
            .direction = Vec3{-0.345923901, 0.519543409, 0.781288147}});
        REQUIRE(!maybe_hit);
    }

    SECTION("Trace a cube with non-identity obb transformation")
    {
        pool.reset(10000, 10000);

        Obb volume{.center = Vec3{0.f}, .orientation = Mat3{1}, .half_extent = Vec3{1.f}};
        i32 max_depth = 3;
        Svo svo{pool, volume, max_depth};

        f32 step = svo.get_voxel_world_size();
        for (f32 x = -1.f; x <= 0.f; x += step) {
            for (f32 y = -1.f; y <= 0.f; y += step) {
                for (f32 z = -1.f; z <= 0.f; z += step) {
                    svo.set_color_at_location({x, y, z}, Vec4{1});
                }
            }
        }

        svo.build_tree();

        SECTION("Rays parallel to an axis")
        {

            auto maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {-0.5f, -0.5f, -5.f},
                                                          .direction = Vec3{0, 0, 1.f}});
            REQUIRE(maybe_hit);

            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {-0.5f, -0.5f, 5.f},
                                                     .direction = Vec3{0, 0, -1.f}});
            REQUIRE(maybe_hit);

            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {0.5f, -0.5f, 5.f},
                                                     .direction = Vec3{0, 0, -1.f}});
            REQUIRE(!maybe_hit);
        }

        SECTION("Rays almost parallel to an axis")
        {
            auto maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {-0.5f, -0.5f, -5.f},
                                                          .direction = glm::normalize(Vec3{-0.0001f, 0.0001f, 1.f})});
            REQUIRE(maybe_hit);

            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {-0.5f, -0.5f, 5.f},
                                                     .direction = glm::normalize(Vec3{-0.0001f, 0.0001f, -1.f})});
            REQUIRE(maybe_hit);

            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {0.5f, -0.5f, 5.f},
                                                     .direction = glm::normalize(Vec3{-0.0001f, 0.0001f, -1.f})});
            REQUIRE(!maybe_hit);
        }

        SECTION("Angled rays")
        {
            auto maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {2.f, 2.f, -0.5f},
                                                          .direction = glm::normalize(Vec3{-1.f, -1.f, 0.f})});
            REQUIRE(maybe_hit);

            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {-2.f, -2.f, -0.5f},
                                                          .direction = glm::normalize(Vec3{1.f, 1.f, 0.f})});
            REQUIRE(maybe_hit);

            maybe_hit = Tracing::trace_svo_ray(svo, {.origin = {-2.f, -2.f, 0.f + step},
                                                          .direction = glm::normalize(Vec3{1.f, 1.f, 0.f})});
            REQUIRE(!maybe_hit);
        }

        SECTION("Watertight rays")
        {
            for (f32 x = 0.00001f; x <= 1.f; x += step * 0.25f) {
                for (f32 y = 0.00001f; y <= 1.f; y += step * 0.25f) {
                    Vec3 target{ -1.f + x, -1.f + y, 0.f };
                    Vec3 offset = Vec3{ 7.f, 3.f, 200.f };

                    auto maybe_hit = Tracing::trace_svo_ray(svo, {.origin = target + offset,
                                                          .direction = glm::normalize(-offset)});
                    REQUIRE(maybe_hit);

                    maybe_hit = Tracing::trace_svo_ray(svo, {.origin = target + offset + Vec3{0.f, -1.f, 0.f},
                                                          .direction = glm::normalize(-offset)});
                    REQUIRE(!maybe_hit);
                }
            }

            for (f32 x = 0.00001f; x <= 1.f; x += step * 0.25f) {
                for (f32 y = 0.00001f; y <= 1.f; y += step * 0.25f) {
                    Vec3 target{ -1.f + x, -1.f, -1.f + y };
                    Vec3 offset = Vec3{ 7.f, -200.f, -10.f };

                    auto maybe_hit = Tracing::trace_svo_ray(svo, {.origin = target + offset,
                                                          .direction = glm::normalize(-offset)});
                    REQUIRE(maybe_hit);

                    maybe_hit = Tracing::trace_svo_ray(svo, {.origin = target + offset + Vec3{-1.f, 0.f, 0.f},
                                                          .direction = glm::normalize(-offset)});
                    REQUIRE(!maybe_hit);
                }
            }
        }
    }

    SECTION("Failing use cases")
    {
        pool.reset(10000, 10000);

        Obb volume{ .center = Vec3{}, .orientation = Mat3{1}, .half_extent = Vec3{1.f} };
        i32 max_depth = 4;
        Svo svo{pool, volume, max_depth};

        f32 step = svo.get_voxel_world_size();
        for(f32 x=-0.5f; x<=0.5f; x+=step) {
            for(f32 y=-0.5f; y<=0.5f; y+=step) {
                for(f32 z=-0.5f; z<=0.5f; z+=step) {
                    if(glm::sqrt(square(x) + square(y) + square(z)) <= 0.5f) {
                        svo.set_color_at_location({x, y, z}, 
                        {   glm::fract(x), 
                            glm::fract(y), 
                            glm::fract(z), 
                            1.f});
                    }
                }
            }
        }
        svo.build_tree();

        SECTION("Ray failing because of messed up tranisition between nodes of different size") {
            Ray ray{.origin= {1, 1, -0.5}, .direction= {-0.548839092, -0.144500583, 0.823343813}};
            auto result = Tracing::trace_svo_ray(svo, ray);
            REQUIRE(!result);
        }

        SECTION("Ray stuck with infinite loop") {
            Ray ray{.origin= {1, 1, -0.5}, .direction= {-0.999220133, 0.00505414605, -0.0391596556}};
            auto result = Tracing::trace_svo_ray(svo, ray);
            REQUIRE(!result);
        }

        SECTION("Ray stuck with infinite loop v2") {
            Ray ray{.origin= {1, 1, -0.5}, .direction= {-0.999983430, -0.00573852658, -0.000237464905}};
            auto result = Tracing::trace_svo_ray(svo, ray);
            REQUIRE(!result);
        }
    }
}
