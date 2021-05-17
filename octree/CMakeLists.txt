add_executable(octree src/main.cpp)
target_link_libraries(octree vk-bootstrap::vk-bootstrap Vulkan::Vulkan fmt::fmt glfw glm::glm)

add_subdirectory(tests)
