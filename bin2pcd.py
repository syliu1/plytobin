import numpy as np
import open3d as o3d
import struct
from open3d import geometry, utility

def numpy_bin_to_pcd(binPath):
    # Load binary point cloud
    bin_pcd = np.fromfile(binPath, dtype=np.float32)

    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1, 4))[:, 0:3]

    # Convert to Open3D point cloud
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    # Save to whatever format you like
    return o3d_pcd

def byte_bin_to_pcd(binFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack(">ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd

print(numpy_bin_to_pcd("A:/kitti-velodyne-viewer/test_out/00000.bin"))
#print(byte_bin_to_pcd("A:/kitti-velodyne-viewer/test_out/00000.bin"))