## PLY to BIN Converter

Convert ply files to bin files

## Requirements

The code requires the PCL library to be installed (https://pointclouds.org/downloads/) and Python 3.6 - 3.9 (Code was tested on Python 3.7)

## Dataset

Ensure all ply files are in the same folder accessible by this repo.

## Convert `.ply` to `.bin`
Usage:

Alter ```python
 INPUT_DIR``` and ```python
 OUTPUT_DIR``` variables to be the path to .ply input folder, and the output folder respectively.

```bash
python ply2bin.py 
```