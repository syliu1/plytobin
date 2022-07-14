import numpy as np
import open3d as o3d
import struct
from open3d import geometry, utility

# Load binary point cloud
bin_pcd = np.fromfile("A:/kitti-velodyne-viewer/test_out/00000.bin", dtype=np.float32)

# Reshape and drop reflection values
points = bin_pcd.reshape((-1, 4))[:, 0:3]

# Convert to Open3D point cloud
o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

# Save to whatever format you like
print(o3d_pcd)
#o3d.io.write_point_cloud("pointcloud.pcd", o3d_pcd)

# def convert_kitti_bin_to_pcd(binFilePath):
#     size_float = 4
#     list_pcd = []
#     with open(binFilePath, "rb") as f:
#         byte = f.read(size_float * 4)
#         while byte:
#             x, y, z, intensity = struct.unpack("ffff", byte)
#             list_pcd.append([x, y, z])
#             byte = f.read(size_float * 4)
#     np_pcd = np.asarray(list_pcd)
#     pcd = geometry.PointCloud()
#     pcd.points = utility.Vector3dVector(np_pcd)
#     return pcd

# print(convert_kitti_bin_to_pcd("A:/kitti-velodyne-viewer/test_out/00000.bin"))