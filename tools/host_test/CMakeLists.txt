# Builds the reconstruction core for the host so the geometry can be exercised
# without an Android device:
#   cmake -S tools/host_test -B build/host && cmake --build build/host -j
#   ./build/host/bulletcam_host_test /tmp/bulletcam_test
cmake_minimum_required(VERSION 3.16)
project(bulletcam_host_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../app/src/main/cpp)

add_executable(bulletcam_host_test
    synthetic_scene.cpp
    android_log_shim.cpp
    ${CORE_DIR}/pipeline.cpp
    ${CORE_DIR}/core/image.cpp
    ${CORE_DIR}/core/frame_store.cpp
    ${CORE_DIR}/sparse/orb.cpp
    ${CORE_DIR}/sparse/sparse_map.cpp
    ${CORE_DIR}/dense/view_selection.cpp
    ${CORE_DIR}/dense/patchmatch.cpp
    ${CORE_DIR}/dense/fusion.cpp
    ${CORE_DIR}/surface/point_cloud.cpp
    ${CORE_DIR}/surface/poisson.cpp
    ${CORE_DIR}/surface/marching_tets.cpp
    ${CORE_DIR}/surface/mesh.cpp
    ${CORE_DIR}/surface/atlas.cpp
    ${CORE_DIR}/texture/texturer.cpp
    ${CORE_DIR}/io/png.cpp
    ${CORE_DIR}/io/model_writer.cpp)

target_include_directories(bulletcam_host_test PRIVATE ${CORE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/shim)
target_compile_options(bulletcam_host_test PRIVATE -O2 -Wall -Wextra -Wno-unused-parameter)

find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
target_link_libraries(bulletcam_host_test PRIVATE ZLIB::ZLIB Threads::Threads)
