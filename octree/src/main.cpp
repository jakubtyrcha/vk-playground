#include <VkBootstrap.h>
#include <fmt/core.h>

#include "window.h"

#include "svo.h"

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