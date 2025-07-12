#pragma once

#include <unordered_map>
#include <Eigen/Core>
#include <vector>

class OccupancyGrid {
public:
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHasher {
        //This is a functor. Here unordered map expects a type (the 3rd argument VoxelKeyHasher )
        //size_t is a type of unsigned integer. used generally to store memory addresses or container sizes. hash functions must return size_t.
        std::size_t operator()(const VoxelKey& key) const {
            //Creates a hash with our key. ^ is bitwise XOR. Think this like encoding. << 1 shifts the number to 2^1(since it is binary).
            return std::hash<int>()(key.x) ^ std::hash<int>()(key.y << 1) ^ std::hash<int>()(key.z << 2);
        }
    };

    OccupancyGrid(double voxel_size);

    void integrate(const std::vector<Eigen::Vector3d>& cloud, const Eigen::Matrix4d& pose);

    std::vector<Eigen::Vector3d> getOccupiedPoints() const;

    std::vector<VoxelKey> bresenham3D(const VoxelKey& start, const VoxelKey& end) const;

    std::vector<Eigen::Vector3d> DownsamplePCD(const std::vector<Eigen::Vector3d>& cloud, double voxel_size);

private:
    
    double m_voxel_size;
    double m_loggodds_occ;
    double m_loggodds_free;
    double m_loggodds_min;
    double m_loggodds_max;
    //very fast access times. good for space.
    std::unordered_map<VoxelKey, double, VoxelKeyHasher> m_grid; // true = occupied
};
