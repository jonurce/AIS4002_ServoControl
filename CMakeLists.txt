cmake_minimum_required(VERSION 3.19)
project(servo_control)

set(CMAKE_CXX_STANDARD 20)

file(COPY "data/" DESTINATION "${CMAKE_BINARY_DIR}/data")

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF)
set(THREEPP_BUILD_EXAMPLES OFF)
FetchContent_Declare(
        threepp
        GIT_REPOSITORY https://github.com/markaren/threepp.git
        GIT_TAG 17a0d47f0d6b9d04e090ef4065ab4806c2deff52
)
FetchContent_MakeAvailable(threepp)

if (WIN32)
    if (VCPKG_TARGET_TRIPLET)
        set(OpenCV_DIR "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")
    else ()
        set(OpenCV_DIR "$ENV{OpenCV_DIR}")
    endif ()
endif()

find_package(OpenCV REQUIRED)


add_executable(servo_control main.cpp)
target_link_libraries(servo_control PRIVATE threepp::threepp ${OpenCV_LIBRARIES})
