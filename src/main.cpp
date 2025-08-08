#include <CLI11.hpp>
#include "../include/OccupancyGrid.hpp"
#include <iostream>
#include "../dataloader/include/dataloader.hpp"
#include "../visualizer/include/visualizer.hpp"



int main(int argc, char** argv) {
    
    double voxel_size = 0.5; //default
    int num_files = 100;     //default
    
    CLI::App app{"Occupancy Grid Map"};

    app.add_option("-v,--voxel", voxel_size, "Voxel size to use during downsampling")
        ->default_val(voxel_size);

    app.add_option("-f,--files", num_files, "Number of PLY files to process")
        ->default_val(num_files);

    CLI11_PARSE(app, argc, argv);
   
    std::cout << voxel_size << std::endl;
    std::cout << num_files << std::endl;
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
    const dataloader::Dataset dataset = dataloader::Dataset("./Data");

    OccupancyGrid grid(voxel_size);

    for (int i = 0; i < num_files; ++i) {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

        //every time you get a pair with poses as the first element and pcd as the second 
        PoseAndCloud data = dataset[i];

        std::vector<Eigen::Vector3d> filtered_pcd = grid.DownsamplePCD(data.second,voxel_size);
        grid.integrate(filtered_pcd, data.first);
        
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

        const double duration = std::chrono::duration<double>(end - start).count();

        std::cout << "Scan " << i << " done in " << duration << " seconds." << std::endl;
    }
    auto getOccupiedPoints = [&grid]() {
        std::vector<Eigen::Vector3d> points;
        const auto& map = grid.getGrid();
    
        points.reserve(map.size());
        for (const auto& [key, value] : map) {
            if (value > 0.0) {
                points.emplace_back(
                    key.x * grid.getVoxelSize(),
                    key.y * grid.getVoxelSize(),
                    key.z * grid.getVoxelSize());
            }
        }
        return points;
    };
    visualize(getOccupiedPoints());

    return 0;
}
