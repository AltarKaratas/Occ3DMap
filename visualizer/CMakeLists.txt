
add_library(visualizer INTERFACE)


target_include_directories(visualizer
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/external/eigen
)
target_link_libraries(visualizer INTERFACE Open3D::Open3D)

