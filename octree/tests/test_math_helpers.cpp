#include <catch2/catch.hpp>

#include "math_helpers.h"

TEST_CASE( "Can move ray to Obb local space", "[math_helpers]" ) {
    {
        Obb obb{{0, 0, 0}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {1, 1, 1}};
        Ray ray{{0, 0, 0}, {1, 0, 0}};

        Ray local_ray = obb.to_local(ray);
        REQUIRE(local_ray == ray);
    }
    {
        Obb obb{{0, 0, 0}, {{0, 0, 1}, {1, 0, 0}, {0, 0, 1}}, {1, 1, 1}};
        Ray ray{{0, 0, 0}, {1, 0, 0}};

        Ray local_ray = obb.to_local(ray);
        REQUIRE(local_ray == Ray{.direction = {0, 1, 0}});
    }
    {
        Obb obb{{2, 2, 0}, {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}, {1, 1, 1}};
        Ray ray{{2, 0, 0}, {1, 0, 0}};

        Ray local_ray = obb.to_local(ray);
        REQUIRE(local_ray == Ray{.origin = {-2, 0, 0}, .direction = {0, -1, 0}});
    }
}