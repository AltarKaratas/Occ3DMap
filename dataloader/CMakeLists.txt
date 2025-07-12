
add_library(dataloader)

target_sources(dataloader PRIVATE dataloader.cpp)

target_include_directories(dataloader
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/external/eigen
)
target_link_libraries(dataloader PUBLIC Open3D::Open3D)

