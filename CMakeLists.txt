cmake_minimum_required(VERSION 3.25)
project(SelectPointPairAdapter)

include(${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")
MESSAGE(STATUS "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_CXX_STANDARD 17)

add_executable(SelectPointPairAdapter main.cpp)

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(SelectPointPairAdapter PRIVATE Eigen3::Eigen)


