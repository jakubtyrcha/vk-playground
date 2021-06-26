option(CATCH_BUILD_TESTING "" OFF)
option(CATCH_ENABLE_WERROR "" OFF)
option(CATCH_INSTALL_DOCS "" OFF)
option(CATCH_INSTALL_HELPERS "" OFF)
set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1) #remove Catch2 target spam

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2
    GIT_TAG        v2.13.6
)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)

set (CMAKE_CXX_STANDARD 20)

add_executable(tests tests.cpp test_math_helpers.cpp test_svo.cpp ../src/math_helpers.cpp)
target_link_libraries(tests glm::glm Catch2::Catch2)
target_include_directories(tests PRIVATE ../src)

include(CTest)
include(Catch)

enable_testing()
catch_discover_tests(tests)

