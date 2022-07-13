"""
Script to convert multiple PCD files into other formats using PCL
"""
import os
import subprocess

# Make sure you have PCL installed
package = 'pcl_convert_pcd_ascii_binary'
input_dir = 'A:/kitti-velodyne-viewer/test'
output_dir = 'A:/kitti-velodyne-viewer/test_out'

# Available conversion formats
asciiFormat = '0'
binaryFormat = '1'
binaryCompressedFormat = '2'

formatChoice = binaryFormat

for pcd in os.listdir(input_dir):
    
    print('Converting pcd :', pcd)
    #inputPCD = input_dir + '/' + pcd
    #outputPCD = output_dir + '/' + 'binComp.bin'

    inputPCD = os.path.join(input_dir, pcd)
    outputPCD = os.path.join(output_dir, os.path.basename(inputPCD))

    # PCL command
    command = [package,
               inputPCD,
               outputPCD,
               formatChoice]

    subprocess.call(command)
    break

os.remove(inputPCD)