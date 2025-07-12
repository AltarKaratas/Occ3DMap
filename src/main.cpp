#include "../include/OccupancyGrid.hpp"
#include <iostream>
#include "../dataloader/include/dataloader.hpp"
#include "../visualizer/include/visualizer.hpp"



int main() {
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
    const dataloader::Dataset dataset = dataloader::Dataset("./Data");

    const double voxel_size = 0.2;
    OccupancyGrid grid(voxel_size);

    for (int i = 0; i < 3; ++i) {
        auto start = std::chrono::high_resolution_clock::now();

        //every time you get a pair with poses as the first element and pcd as the second 
        PoseAndCloud data = dataset[i];

        auto filtered_pcd = grid.DownsamplePCD(data.second,voxel_size);
        grid.integrate(filtered_pcd, data.first);
        
        auto end = std::chrono::high_resolution_clock::now();

        double duration = std::chrono::duration<double>(end - start).count();

        std::cout << "Scan " << i << " done in " << duration << " seconds." << std::endl;
    }
    
    visualize(grid.getOccupiedPoints());

    return 0;
}
