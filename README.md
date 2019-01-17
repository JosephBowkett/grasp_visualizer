# grasp_visualizer
Simple tool to visualize a grasp pose within a pointcloud

## Build

Requires PCL >=1.7 from [here](https://github.com/PointCloudLibrary/pcl/tree/pcl-1.7.2)

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
grasp_visualizer pcloud.pcd pose.txt
```

To display a gripper in the pointcloud:
```bash
grasp_visualizer pcloud.pcd pose.txt -g gripper.ply
```

To additionally display a box shaped region of interest:
```bash
grasp_visualizer pcloud.pcd pose.txt -r roi.txt
```

Where `pose.txt` has the format:
```bash
x y z qw qx qy qz
```

Where `roi.txt` has the format:
```bash
x y z qw qx qy qz xdim ydim zdim
```