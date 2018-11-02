# grasp_visualizer
Simple tool to visualize a grasp pose within a pointcloud

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