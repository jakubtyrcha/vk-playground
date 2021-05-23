#include "math_helpers.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

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
};

int main() {
    Vec2i resolution{800, 600};
    Vec3 camera_eye{0, 0, -3};
    Vec3 camera_dir{0, 0, 1};
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

    for(i32 y=0; y<resolution.y; y++) {
        for(i32 x=0; x<resolution.x; x++) {
            const Vec2 clip_space = ((Vec2{x, y} + 0.5f) / Vec2{ resolution }) * Vec2{-2, 2} + Vec2{1, -1};
            
            Vec4 view_camera_ray_homog = Vec4{clip_space, 1, 1} * inv_projection_mat;
            Vec3 view_camera_ray = glm::normalize(Vec3{view_camera_ray_homog / view_camera_ray_homog.w});

            Vec3 world_camera_ray = view_camera_ray * Mat3{inv_view_mat};

            Vec2 barycentric;
            f32 t;
            if(glm::intersectRayTriangle(camera_eye, world_camera_ray, triangle_pos[0], triangle_pos[1], triangle_pos[2], barycentric, t)) {
                Vec3 col = (1-barycentric.x - barycentric.y) * triangle_col[0] + barycentric.x * triangle_col[1] + barycentric.y * triangle_col[2];
                image_buffer.store_color({x, y}, Vec4{col, 1});
            }
            else {
                image_buffer.store_color({x, y}, Vec4{0, 0, 0, 1});
            }
        }
    }

    stbi_write_png("image.png", resolution.x, resolution.y, 4, image_buffer.data(), image_buffer.get_stride());

    return 0;
}