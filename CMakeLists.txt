cmake_minimum_required(VERSION 3.24)
project(occ_grid_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Open3D REQUIRED)


add_subdirectory(dataloader)
add_subdirectory(visualizer)

add_executable(main_exec)

target_include_directories(main_exec PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/external/eigen)

target_sources(main_exec PRIVATE src/main.cpp src/OccupancyGrid.cpp)

target_link_libraries(main_exec
    PRIVATE
    dataloader
    visualizer
    Open3D::Open3D
)
