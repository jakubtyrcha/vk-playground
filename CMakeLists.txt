cmake_minimum_required(VERSION 3.20)
project(VkPlayground)

set (CMAKE_CXX_STANDARD 20)

find_package(Vulkan REQUIRED)

add_subdirectory(ext)
add_subdirectory(octree)