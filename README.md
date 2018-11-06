# grasp_visualizer
Simple tool to visualize a grasp pose within a pointcloud

## Build

Requires PCL >1.8 from [here](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.8.0)

To Build:
```bash
git clone https://github.com/JosephBowkett/grasp_visualizer.git
cd grasp_visualizer
mkdir build && cd build
cmake ..
make
```
## Lazy Install (if desired)

```bash
echo "export PATH=\$PATH:$PWD" >> ~/.bashrc
```

## Usage

To display just a pose in the pointcloud:
```bash
grasp_visualizer -p [pcloud.pcd] [pose.txt]
```

To display a gripper in the pointcloud:
```bash
grasp_visualizer -g [pcloud.pcd] [pose.txt] [gripper.ply]
```

Where `[pose.txt]` has the format:
```bash
x y z qw qx qy qz
```