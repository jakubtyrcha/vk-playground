#pragma once

#if defined(_WIN32)
#include <fcntl.h>
#define NOMINMAX
#include <windows.h>
#endif // _WIN32

#if defined(__linux__) || defined(__APPLE__)
#include <dlfcn.h>
#endif

#define GLFW_INCLUDE_VULKAN
#include "GLFW/glfw3.h"

#include <fmt/core.h>

GLFWwindow* create_window_glfw(const char* window_name = "", bool resize = true) {
	glfwInit();
	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	if (!resize) glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

	return glfwCreateWindow(1024, 1024, window_name, NULL, NULL);
}
void destroy_window_glfw(GLFWwindow* window) {
	glfwDestroyWindow(window);
	glfwTerminate();
}
VkSurfaceKHR create_surface_glfw(VkInstance instance, GLFWwindow* window) {
	VkSurfaceKHR surface = VK_NULL_HANDLE;
	VkResult err = glfwCreateWindowSurface(instance, window, NULL, &surface);
	if (err) {
		const char* error_msg;
		int ret = glfwGetError(&error_msg);
		if (ret != 0) {
			fmt::print("Failed to create glfw window with error {}", ret);
			if (error_msg != nullptr) fmt::print("Error msg: {}", error_msg);
		}
		surface = VK_NULL_HANDLE;
	}
	return surface;
}

