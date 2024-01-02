cmake_minimum_required(VERSION 3.26)

set(PROJECT_NAME PGO_STUDY)
project(${PROJECT_NAME} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUITED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (default Release)" FORCE)
endif()
message("CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

find_package(Open3D REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/open3d/lib/cmake/Open3D)
find_package(OpenMP REQUIRED)

add_executable(pose_graph_3d 
    src/main.cpp
    src/math.cpp
    src/read_g2o.cpp
    src/types.cpp
    src/pose_graph_optimizer.cpp
)
include_directories(include)
target_link_libraries(pose_graph_3d PRIVATE 
    Open3D::Open3D
    OpenMP::OpenMP_CXX
)
target_compile_definitions(pose_graph_3d PRIVATE
    G2O_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data/sphere.g2o"
)
