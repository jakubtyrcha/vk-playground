#include "math_helpers.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "svo.h"
#include <filesystem>
#include <fmt/core.h>

using u8Vec4 = glm::u8vec4;

struct ImageBuffer {
    using PixelType = u8Vec4;
    std::vector<PixelType> data_;
    Vec2i resolution_;

    ImageBuffer(Vec2i resolution) : resolution_(resolution) {
        data_.resize(resolution.x * resolution.y);
    }

    const PixelType* data() const {
        return data_.data();
    }

    i32 get_stride() const {
        return sizeof(PixelType) * resolution_.x;
    }

    void clear(u8Vec4 color) {
        const i32 N = resolution_.x * resolution_.y;
        for(i32 i=0; i<N; i++) {
            data_[i] = color;
        }
    }

    void store_color(Vec2i texel, Vec4 color) {
        u8Vec4 qcolor = glm::clamp(color * Vec4{255}, {}, {255});
        data_[texel.x + texel.y * resolution_.x] = qcolor;
    }

    void save_to_file(const char * filename) {
        stbi_write_png(filename, resolution_.x, resolution_.y, 4, data(), get_stride());
    }
};

int main() {
    Vec2i resolution{800, 600};
    Vec3 camera_eye{0, 0, -1};
    Vec3 camera_dir = glm::normalize(Vec3{0, 0, 1});
    Vec3 camera_up{0, 1, 0};
    f32 camera_fov_y = glm::half_pi<f32>();

    ImageBuffer image_buffer{resolution};
    image_buffer.clear({ 0, 0, 0, 255});

    Mat4 view_mat = glm::lookAtLH(camera_eye, camera_eye + camera_dir, camera_up);
    Mat4 inv_view_mat = glm::inverse(view_mat);
    Mat4 projection_mat = glm::perspectiveFovLH_ZO(camera_fov_y, (f32)resolution.x, (f32)resolution.y, 1.f, 100.f);
    Mat4 inv_projection_mat = glm::inverse(projection_mat);

    Vec3 triangle_pos[3] = { {-2.5, -2, 1.5f}, {3.5, 2.f, 1.25f}, { -3.5f, 2.5f, 1.f } };
    Vec3 triangle_col[3] = { { 1, 0, 0 }, {0, 1, 0}, {0, 0, 1} };

    SvoPool<4, BrickVoxelPosition::NodeCorner> pool;
    pool.reset(10000, 10000);

    Obb volume{ .center = Vec3{}, .orientation = Mat3{1}, .half_extent = Vec3{1.f} };
    i32 max_depth = 3;
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

    for(i32 y=0; y<resolution.y; y++) {
        for(i32 x=0; x<resolution.x; x++) {
            const Vec2 framebuffer_coord = Vec2{x, y} + 0.5f;
            const Vec3 screen_ray_ndf_coord = {(framebuffer_coord / Vec2{ resolution }) * Vec2{2, -2} + Vec2{-1, 1}, 1};

            Vec4 camera_ray_dir_viewspace_homog = inv_projection_mat * Vec4{screen_ray_ndf_coord, 1};
            // todo: wtf is this negative
            Vec3 camera_ray_dir_viewspace = glm::normalize(Vec3{camera_ray_dir_viewspace_homog / camera_ray_dir_viewspace_homog.w});

            Vec3 camera_ray_dir_worldspace = Mat3{inv_view_mat} * camera_ray_dir_viewspace;

#if 0
            Vec2 barycentric;
            f32 t;
            if (glm::intersectRayTriangle(camera_eye, camera_ray_dir_worldspace, triangle_pos[0], triangle_pos[1], triangle_pos[2], barycentric, t))
            {
                Vec3 col = (1 - barycentric.x - barycentric.y) * triangle_col[0] + barycentric.x * triangle_col[1] + barycentric.y * triangle_col[2];
                image_buffer.store_color({x, y}, Vec4{col, 1});
            }
            else
            {
                image_buffer.store_color({x, y}, Vec4{0, 0, 0, 1});
            }
#elif 1
            Ray ray{.origin = camera_eye, .direction = camera_ray_dir_worldspace};
            if(auto result = Tracing::trace_svo_ray(svo, ray); result) {
                image_buffer.store_color({x, y}, {Vec3{result->color}, 1});
            }
            else {
                image_buffer.store_color({x, y}, Vec4{0, 0, 0, 1});
            }
#else
            image_buffer.store_color({x, y}, Vec4{camera_ray_dir_worldspace, 1});
#endif
            
        }
    }

    image_buffer.save_to_file("image.png");
    fmt::print("file saved to {}", std::filesystem::current_path().string());

    return 0;
}