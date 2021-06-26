set(IMGUI_DIR ../ext/imgui)

add_executable(window src/main.cpp src/math_helpers.cpp ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp ${IMGUI_DIR}/backends/imgui_impl_vulkan.cpp ${IMGUI_DIR}/imgui.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp)
target_link_libraries(window vk-bootstrap::vk-bootstrap Vulkan::Vulkan fmt::fmt glfw glm::glm)
target_include_directories(window PRIVATE ${IMGUI_DIR} ${IMGUI_DIR}/backends)

add_executable(trace src/trace.cpp src/math_helpers.cpp)
target_link_libraries(trace glm::glm fmt::fmt)
target_include_directories(trace PRIVATE ../ext/stb)

add_subdirectory(tests)
