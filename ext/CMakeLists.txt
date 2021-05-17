option(GLFW_BUILD_TESTS "" OFF)
option(GLFW_BUILD_DOCS "" OFF)
option(GLFW_INSTALL "" OFF)
option(GLFW_BUILD_EXAMPLES "" OFF)

option(FMT_INSTALL "" OFF)
option(FMT_DOC "" OFF)
option(FMT_TEST "" OFF)

include(FetchContent)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw
    GIT_TAG        3.3.2
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    Glm
    GIT_REPOSITORY https://github.com/g-truc/glm
    GIT_TAG        0.9.9.8
)
FetchContent_MakeAvailable(Glm)

FetchContent_Declare(
    VkBootstrap
    GIT_REPOSITORY https://github.com/charles-lunarg/vk-bootstrap
    GIT_TAG        v0.4
)
FetchContent_MakeAvailable(VkBootstrap)

FetchContent_Declare(
    Fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt
    GIT_TAG        7.1.3
)
FetchContent_MakeAvailable(Fmt)