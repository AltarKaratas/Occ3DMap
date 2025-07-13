#include "../include/OccupancyGrid.hpp"
#include <iostream>
#include "../dataloader/include/dataloader.hpp"
#include "../visualizer/include/visualizer.hpp"



int main(int argc, char** argv) {
    
    double voxel_size = 0.3; //default
    int num_files = 100;     //default

    if(argc == 1){
        std::cout << "Usage: build/main_exec --voxel_size <value> --num_files <value>\n";
        return 0;
    }    

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--voxel_size" && i + 1 < argc) {
            voxel_size = std::stod(argv[++i]);
            std::cout<<"girdi"<<std::endl;
        }
        else if (arg == "--num_files" && i + 1 < argc) {
            num_files = std::stoi(argv[++i]);
            std::cout<<"girdi"<<std::endl;

        }
        else if (arg == "--help") {
            std::cout << "Usage: build/main_exec --voxel_size <value> --num_files <value>\n";
            return 0;
        }
    }

    std::cout << "Voxel size: " << voxel_size << "\n";
    std::cout << "Number of files to process: " << num_files << "\n";

   
    using Vector3dVector = std::vector<Eigen::Vector3d>;
    using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;
    const dataloader::Dataset dataset = dataloader::Dataset("./Data");

    OccupancyGrid grid(voxel_size);

    for (int i = 0; i < num_files; ++i) {
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
