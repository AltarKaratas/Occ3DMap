#include "../include/OccupancyGrid.hpp"
#include <chrono>
#include <iostream>
#include <unordered_set>

OccupancyGrid::OccupancyGrid(double voxel_size)
    : m_voxel_size(voxel_size),
      m_loggodds_occ(0.8),
      m_loggodds_free(-0.4),
      m_loggodds_min(-2.0),
      m_loggodds_max(3.5)
{
}

void OccupancyGrid::integrate(const std::vector<Eigen::Vector3d> &cloud, const Eigen::Matrix4d &pose)
{

    // block means get a chunk from position 0,3 -> block is sized 3,1 -> this is the translation part of the matrix4d poses
    //  imagine like you are drawing a ray out of camera to each hit point of the lidar sensor.
    Eigen::Vector3d sensor_origin = pose.block<3, 1>(0, 3);

    std::unordered_set<VoxelKey, VoxelKeyHasher> visited_voxels;

    for (const auto &pt : cloud)
    {
        // Sensor origin -> voxel
        VoxelKey start_voxel = {
            static_cast<int>(std::floor(sensor_origin.x() / m_voxel_size)),
            static_cast<int>(std::floor(sensor_origin.y() / m_voxel_size)),
            static_cast<int>(std::floor(sensor_origin.z() / m_voxel_size))};

        // Point to world
        Eigen::Vector4d homo_pt(pt.x(), pt.y(), pt.z(), 1.0);
        // this is a matrix mul
        Eigen::Vector4d pt_in_map = pose * homo_pt;

        VoxelKey end_voxel = {
            static_cast<int>(std::floor(pt_in_map.x() / m_voxel_size)),
            static_cast<int>(std::floor(pt_in_map.y() / m_voxel_size)),
            static_cast<int>(std::floor(pt_in_map.z() / m_voxel_size))};

        // bresenham
        // this returns a vector full of VoxelKeys that our rays have passed.
        auto bres_line = bresenham3D(start_voxel, end_voxel);

        // We remove the last voxel (the hit point) from the Bresenham line before applying the 'free' update.
        // This is important because the hit voxel should be updated as 'occupied', not 'free'.
        // If we don't exclude it, it will be updated twice — first as free (along the ray path),
        // then as occupied (as the endpoint). This weakens the log-odds confidence of that occupied voxel.
        if (!bres_line.empty())
            bres_line.pop_back();

        // for every voxel ray passed:
        for (const auto &voxel : bres_line)
        {
            // find that in grid -> clamp -> keeps the value between min and max values. since these are empty voxels -> update empty cells
            if (visited_voxels.find(voxel) == visited_voxels.end())
            {
                m_grid[voxel] = std::clamp(m_grid[voxel] + m_loggodds_free,
                                           m_loggodds_min, m_loggodds_max);
                visited_voxels.insert(voxel);
            }
        }
        // update the occupied cells' value
        m_grid[end_voxel] = std::clamp(m_grid[end_voxel] + m_loggodds_occ,m_loggodds_min, m_loggodds_max);
    }
}

std::vector<Eigen::Vector3d> OccupancyGrid::getOccupiedPoints() const
{
    std::vector<Eigen::Vector3d> points;
    for (const auto &[key, value] : m_grid)
    {
        if (value > 0.0)
        {
            points.emplace_back(
                key.x * m_voxel_size,
                key.y * m_voxel_size,
                key.z * m_voxel_size);
        }
    }
    return points;
}

std::vector<OccupancyGrid::VoxelKey> OccupancyGrid::bresenham3D(const VoxelKey &start, const VoxelKey &end) const
{

    std::vector<VoxelKey> line;

    // starting and ending points
    int x0 = start.x;
    int y0 = start.y;
    int z0 = start.z;
    int x1 = end.x;
    int y1 = end.y;
    int z1 = end.z;

    // in which way is the change the largest
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int dz = std::abs(z1 - z0);

    // you start at a point x0,y0,z0 -> x1,y1,z1 -> until this point you have to go dx + dy + dz steps at maximum to avoid memory reallocation
    line.reserve(dx + dy + dz);

    // which way is it going
    int sx = (x1 > x0) ? 1 : -1; // 1 means right -1 means left
    int sy = (y1 > y0) ? 1 : -1;
    int sz = (z1 > z0) ? 1 : -1;

    int dx2 = dx << 1; // there is a trick here starts with 1 and bitshifts left 0001 0010 0100 -> 2-4-8 -> effectively mult. by 2
    int dy2 = dy << 1;
    int dz2 = dz << 1;

    if (dx >= dy && dx >= dz)
    {
        int err_y = dy2 - dx;
        int err_z = dz2 - dx;
        for (int i = 0; i <= dx; ++i)
        {
            line.push_back({x0, y0, z0});
            if (err_y > 0)
            {
                y0 += sy;
                err_y -= dx2;
            }
            if (err_z > 0)
            {
                z0 += sz;
                err_z -= dx2;
            }
            err_y += dy2;
            err_z += dz2;
            x0 += sx;
        }
    }
    else if (dy >= dx && dy >= dz)
    {
        int err_x = dx2 - dy;
        int err_z = dz2 - dy;
        for (int i = 0; i <= dy; ++i)
        {
            line.push_back({x0, y0, z0});
            if (err_x > 0)
            {
                x0 += sx;
                err_x -= dy2;
            }
            if (err_z > 0)
            {
                z0 += sz;
                err_z -= dy2;
            }
            err_x += dx2;
            err_z += dz2;
            y0 += sy;
        }
    }
    else
    {
        int err_x = dx2 - dz;
        int err_y = dy2 - dz;
        for (int i = 0; i <= dz; ++i)
        {
            line.push_back({x0, y0, z0});
            if (err_x > 0)
            {
                x0 += sx;
                err_x -= dz2;
            }
            if (err_y > 0)
            {
                y0 += sy;
                err_y -= dz2;
            }
            err_x += dx2;
            err_y += dy2;
            z0 += sz;
        }
    }

    return line;
}

std::vector<Eigen::Vector3d> OccupancyGrid::DownsamplePCD(const std::vector<Eigen::Vector3d> &cloud, double voxel_size)
{
    std::unordered_map<
        OccupancyGrid::VoxelKey,
        Eigen::Vector3d,
        OccupancyGrid::VoxelKeyHasher>
        voxel_map;

    for (const auto &pt : cloud)
    {
        int ix = static_cast<int>(std::floor(pt.x() / voxel_size));
        int iy = static_cast<int>(std::floor(pt.y() / voxel_size));
        int iz = static_cast<int>(std::floor(pt.z() / voxel_size));

        OccupancyGrid::VoxelKey key{ix, iy, iz};

        // a .find function returns the .end() iterator if not found.
        if (voxel_map.find(key) == voxel_map.end())
        {
            voxel_map[key] = pt;
        }
    }

    std::vector<Eigen::Vector3d> downsampled;
    for (const auto &[key, pt] : voxel_map)
    {
        downsampled.push_back(pt);
    }

    return downsampled;
}