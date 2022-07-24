import os
import subprocess
import open3d as o3d
import shutil 
from pathlib import Path
import numpy as np
from pypcd import pypcd
from tqdm import tqdm
import struct

PACKAGE = 'pcl_convert_pcd_ascii_binary'
PCD_DIR = './pcd_files/'

INPUT_DIR = 'A:/kitti-velodyne-viewer/ply'
OUTPUT_DIR = 'A:/kitti-velodyne-viewer/test_out'

FORMAT_CHOICE = '1'

def ply2pcd_o3d():
    if not os.path.exists(PCD_DIR):
        os.mkdir(PCD_DIR)

    if not os.path.exists(OUTPUT_DIR):
        os.mkdir(OUTPUT_DIR)

    for ply in os.listdir(INPUT_DIR):
        
        pcd = o3d.io.read_point_cloud(os.path.join(INPUT_DIR, ply))
        o3d.io.write_point_cloud(f"{PCD_DIR}{Path(ply).stem}.pcd", pcd)

def pcd2bin_direct():
    
    # Collect pcd files
    pcd_files = []
    for (path, dir, files) in os.walk(PCD_DIR):
        for filename in files:
            ext = os.path.splitext(filename)[-1]
            if ext == '.pcd':
                pcd_files.append(path + "/" + filename)
    pcd_files.sort()   

    print("Starting Conversion:")

    for pcd_file in tqdm(pcd_files):
        # Get pcd file
        pc = pypcd.PointCloud.from_path(pcd_file)

        # Generate bin file name
        bin_file_name = f"{Path(pcd_file).stem}.bin"
        bin_file_path = os.path.join(OUTPUT_DIR, bin_file_name)
        
        # Pull data from ply
        np_x = (np.array(pc.pc_data['x'], dtype=np.float32)).astype(np.float32)
        np_y = (np.array(pc.pc_data['y'], dtype=np.float32)).astype(np.float32)
        np_z = (np.array(pc.pc_data['z'], dtype=np.float32)).astype(np.float32)
        #np_i = (np.array(pc.pc_data['I'], dtype=np.float32)).astype(np.float32)/256

        # Stack all data    
        points_32 = np.transpose(np.vstack((np_x, np_y, np_z)))
        print(points_32.shape)
        # Save bin file                                    
        points_32.tofile(bin_file_path)

def pcd2bin_pcl_lib():
    
    for pcd in os.listdir(PCD_DIR):
        
        print('Converting ply :', pcd)

        inputPCD = os.path.join(PCD_DIR, f"{Path(pcd).stem}.pcd")
        outputPCD = os.path.join(OUTPUT_DIR, f"{Path(inputPCD).stem}.bin")
        
        # Convert to bin
        command = [PACKAGE,
                inputPCD,
                outputPCD,
                FORMAT_CHOICE]

        subprocess.call(command)

    # Cleanup
    os.remove(inputPCD)

def check_points():
    
    '''
    size_float = 4

    for ply in os.listdir(OUTPUT_DIR):
        #point_cloud = o3d.io.read_point_cloud(os.path.join(INPUT_DIR, ply))
        #print(point_cloud)
        
        list_pcd = []

        file = os.path.join(OUTPUT_DIR, ply)
        
        with open (file, "rb") as f:
            byte = f.read(size_float*4)
            while byte:
                x, y, z, i = struct.unpack("ffff", byte)
                list_pcd.append([x, y, z])
                byte = f.read(size_float*4)

        np_pcd = np.asarray(list_pcd)
        pcd = o3d.geometry.PointCloud()
        v3d = o3d.utility.Vector3dVector
        pcd.points = v3d(np_pcd)
        print(f"{Path(ply).stem}: {pcd}")
    '''
    for ply in os.listdir(OUTPUT_DIR):
        
        file = os.path.join(OUTPUT_DIR, ply)
        bin_pcd = np.fromfile(file, dtype=np.float32)
        
        # Reshape and drop reflection values
        #points = bin_pcd.reshape((-1, 4))[:, 0:3]
        points = bin_pcd.reshape((-1, 4))

        # Convert to Open3D point cloud
        o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

        # Save to whatever format you like
        #o3d.io.write_point_cloud("pointcloud.pcd", o3d_pcd)
        print(f"{file}: {o3d_pcd}")

def cleanup():
    shutil.rmtree(PCD_DIR)

if __name__=='__main__':
    ply2pcd_o3d()
    #pcd2bin_direct()
    #pcd2bin_pcl_lib()
    check_points()