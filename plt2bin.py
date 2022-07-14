import os
import subprocess
import open3d as o3d
import shutil 
from pathlib import Path

PACKAGE = 'pcl_convert_pcd_ascii_binary'
PCD_DIR = './pcd_files/'

INPUT_DIR = 'A:/kitti-velodyne-viewer/ply'
OUTPUT_DIR = 'A:/kitti-velodyne-viewer/test_out'

formatChoice = '1'

if not os.path.exists(PCD_DIR):
    os.mkdir(PCD_DIR)

for ply in os.listdir(INPUT_DIR):
    
    pcd = o3d.io.read_point_cloud(os.path.join(INPUT_DIR, ply))
    o3d.io.write_point_cloud(f"{PCD_DIR}{Path(ply).stem}.pcd", pcd)

for pcd in os.listdir(PCD_DIR):
    
    print('Converting ply :', pcd)

    inputPCD = os.path.join(PCD_DIR, f"{Path(pcd).stem}.pcd")
    outputPCD = os.path.join(OUTPUT_DIR, f"{Path(inputPCD).stem}.bin")
    
    # PCL command
    command = [PACKAGE,
               inputPCD,
               outputPCD,
               formatChoice]

    subprocess.call(command)
    break

# Cleanup
os.remove(inputPCD)
shutil.rmtree("./pcd_files")