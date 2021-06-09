add_executable(octree src/main.cpp)
target_link_libraries(octree vk-bootstrap::vk-bootstrap Vulkan::Vulkan fmt::fmt glfw glm::glm)

add_executable(trace src/trace.cpp src/math_helpers.cpp)
target_link_libraries(trace glm::glm fmt::fmt)
target_include_directories(trace PRIVATE ../ext/stb)

add_subdirectory(tests)
