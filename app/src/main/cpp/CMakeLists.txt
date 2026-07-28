cmake_minimum_required(VERSION 3.22.1)
project(bulletcam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BULLETCAM_USE_XATLAS "Use xatlas for UV parameterisation (fetched from GitHub at configure time)" OFF)

add_library(bulletcam SHARED
    bullet_jni.cpp
    pipeline.cpp
    core/image.cpp
    core/frame_store.cpp
    sparse/orb.cpp
    sparse/sparse_map.cpp
    dense/view_selection.cpp
    dense/patchmatch.cpp
    dense/fusion.cpp
    surface/point_cloud.cpp
    surface/poisson.cpp
    surface/marching_tets.cpp
    surface/mesh.cpp
    surface/atlas.cpp
    texture/texturer.cpp
    io/png.cpp
    io/model_writer.cpp)

target_include_directories(bulletcam PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(bulletcam PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -ffast-math
    -fno-finite-math-only
    -funroll-loops)

target_compile_definitions(bulletcam PRIVATE $<$<CONFIG:Release>:BULLETCAM_RELEASE=1>)

if (BULLETCAM_USE_XATLAS)
    include(FetchContent)
    FetchContent_Declare(
        xatlas
        GIT_REPOSITORY https://github.com/jpcy/xatlas.git
        GIT_TAG master
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(xatlas)
    add_library(xatlas_static STATIC ${xatlas_SOURCE_DIR}/source/xatlas/xatlas.cpp)
    target_include_directories(xatlas_static PUBLIC ${xatlas_SOURCE_DIR}/source/xatlas)
    target_compile_options(xatlas_static PRIVATE -Wno-everything)
    target_link_libraries(bulletcam PRIVATE xatlas_static)
    target_compile_definitions(bulletcam PRIVATE BULLETCAM_HAVE_XATLAS=1)
endif ()

find_library(log-lib log)
find_library(z-lib z)

target_link_libraries(bulletcam PRIVATE ${log-lib} ${z-lib})
