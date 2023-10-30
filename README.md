# <img src="https://github.com/woo-soojin/no_more_potentially_dynamic_objects/assets/73815549/29e620d4-c96c-4929-88f8-4201c29a8aa1" width=30/> No More Potentially Dynamic Objects: Static Point Cloud Map Generation based on 3D Object Detection and Ground Projection
Video [[youtube]](https://youtu.be/u_38A-sj_14?si=VN7Y78UUB_vKdiNe)
## Download datasets
KITTI dataset (https://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d).
- velodyne laser data (*.bin) and clibration files (times.txt).

## Verions
We tested the code on *python 3.8* and *Ubuntu 20.04 (ROS noetic)*.

## Setup directories
```bash
bash setup_directories.sh
```

### Unzip and move downloaded data into project directory
Change (`~/dev/no_more_potentially_dynamic_objects`) to your project directory.
```bash
cd ~/Downloads/ ## change directory to the download directory
unzip ./data_odometry_velodyne.zip
mv ./dataset/sequences/00/velodyne ~/dev/no_more_potentially_dynamic_objects/data/KITTI/origin 
unzip ./data_odometry_calib.zip
mv ./dataset/sequences/00/calib/times.txt  ~/dev/no_more_potentially_dynamic_objects/data/KITTI
```

## Setup the environment
```bash
cd ~/dev/no_more_potentially_dynamic_objects ## change directory to the project directory
conda create -n no_more_dynamic python=3.8
conda activate no_more_dynamic
pip install -r requirements.txt
```

## Publish point clouds
```bash
roscore
```
Open the different terminal (no anaconda environment is necessary. Instead, rospkg needs to be installed.)
```bash
python mini_kitti_publisher.py --dir ./data/KITTI
```

## Object detection
We used [VoxelNeXt](https://github.com/dvlab-research/VoxelNeXt) as an object detection method in our paper. For the environment setting, we followed documents on [OpenPCDet](https://github.com/open-mmlab/OpenPCDet).

Additionally, we visualized bounding boxes to test the inference results. To use visualization code, we prepared corresponding pcd and pkl file.
```bash
python visualization.py
```

## Ground segmentation
We used [patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus) as an object detection method in our paper.
Follow the setup process in [patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus)
```bash
python ground_segmentation.py --dir ./data/KITTI
```

## Ground projection
```bash
python ground_projection.py --dir ./data/KITTI
```

## Mapping
### Publish point clouds with projected objects
```bash
python mini_kitti_publisher.py --dir ./data/KITTI --velodyne_dir projected
```
### Proceed mapping
We used [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM) as a mapping method in our paper.


## Acknowledgement
The code in this repository is based on [VoxelNeXt](https://github.com/dvlab-research/VoxelNeXt), [patchwork-plusplus](https://github.com/url-kaist/patchwork-plusplus), [SC-A-LOAM](https://github.com/gisbi-kim/SC-A-LOAM), and [mini-kitti-publisher](https://github.com/gisbi-kim/mini-kitti-publisher). Thanks to the authors of those codes.

## License

Copyright 2023, Soojin Woo, Donghwi Jung, Seong-Woo Kim, Autonomous Robot Intelligence Lab, Seoul National University.

This project is free software made available under the MIT License. For details see the LICENSE file.
