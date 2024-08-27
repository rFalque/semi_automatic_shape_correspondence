# Manual alignement between two shapes

Changelog
- Added the conputation of the normal, and the removal of the correspondences with negative normal dot product (this could fail for pointclouds, need to update it)

## manual annotations
Install open3d:

```bash
pip install open3d
```

or if you have `python` and `python3`

```bash
pip3 install open3d
```

Update the path in the python script if the files have different names in line 5 and 6.

Run two instances of the manual annotations:
```bash
python3 open3d_manual_annotation_CT.py
python3 open3d_manual_annotation_depth.py
```

Instructions for annotations:
Press [shift + left click] for selecting points on each window
Press [shift + right click] to undo the last point picking
After picking points, press 'Q' or close the window
Note: use '1', '2', or '3' to change the gradient orientation
Note: use [shift + +/-] to change the point size

The correspondences will be saved in the data folder with the names:
- CT.csv
- depth.csv

## Moprhing step with embedded deformation
```bash
cd embedded_deformation
mkdir build
cd build
cmake ..
make -j3
```

more info in the [readme](embedded_deformation/README.md).

## to run the deformation
Update the mesh to deform in the [config.yaml](embedded_deformation/config.yaml) file and the output for the formed mesh (lines 2 to 7).
```bash
./embedded_deformation_sample
```
