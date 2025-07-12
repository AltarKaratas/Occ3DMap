import open3d as o3d
import numpy as np
import os
from tqdm import tqdm

def read_poses(filename):
    poses = []
    with open(filename, "r") as f:
        for line in f:
            vals = list(map(float, line.strip().split()))
            if len(vals) == 12:
                pose = np.eye(4)
                pose[:3, :4] = np.array(vals).reshape(3, 4)
                poses.append(pose)
    return poses

def main(data_dir, max_files=None):
    ply_dir = os.path.join(data_dir, "PLY")
    pose_file = os.path.join(data_dir, "gt_poses.txt")

    # Read poses
    poses = read_poses(pose_file)

    # Read PLY file paths
    ply_files = sorted([
        os.path.join(ply_dir, f)
        for f in os.listdir(ply_dir)
        if f.endswith(".ply")
    ])

    if max_files:
        ply_files = ply_files[:max_files]
        poses = poses[:max_files]

    assert len(poses) == len(ply_files), "Mismatch between poses and point clouds!"

    combined_pcd = o3d.geometry.PointCloud()

    for i in tqdm(range(len(ply_files)), desc="Loading and transforming point clouds"):
        pcd = o3d.io.read_point_cloud(ply_files[i])
        pcd.transform(poses[i])
        combined_pcd += pcd
    
    combined_pcd = combined_pcd.voxel_down_sample(voxel_size=0.2)
    print(f"Total points: {len(combined_pcd.points)}")
    o3d.visualization.draw_geometries([combined_pcd])

if __name__ == "__main__":
    # Use the relative or absolute path to your ./Data directory
    main(data_dir="./Data", max_files=None)  # You can change max_files to None to load all
