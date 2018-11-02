/************************************
 *
 * Author: Joseph Bowkett
 * Email:  jbowkett@caltech.edu
 *
 * Grasp Visualizer:
 * Displays pose in pointcloud with
 * option to display gripper model
 *
 **********************************/


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h                                         This help\n"
            << "-p [pcloud.pcd] [pose.txt]                 Display pose in pointcloud\n"
            << "-g [pcloud.pcd] [pose.txt] [gripper.ply]   Display gripper in pointcloud\n\n"
            << "[pose.txt] in format \"x y z qw qx qy qz\"\n"
            << "\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_pose (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, Eigen::Affine3d world_to_grasp)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*cloud, *cloud_transformed, world_to_grasp.inverse());

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_transformed);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_transformed, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.2);
  viewer->initCameraParameters ();

  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualize_gripper (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, Eigen::Affine3d world_to_grasp, pcl::PolygonMesh::Ptr gripper_ptr)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*cloud, *cloud_transformed, world_to_grasp.inverse());

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_transformed);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud_transformed, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->initCameraParameters ();

  viewer->addPolygonMesh(*gripper_ptr);
  return (viewer);
}


int
main (int argc, char** argv)
{
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
  bool display_pose(false), display_gripper(false);
  if ( (pcl::console::find_argument (argc, argv, "-p") >= 0) && (argc == 4) )
  {
    display_pose = true;
    std::cout << "\nDisplaying pose from " << argv[3] << " in PCD " << argv[2] << "\n\n";
  }
  else if ( (pcl::console::find_argument (argc, argv, "-g") >= 0) && (argc == 5) )
  {
    display_gripper = true;
    std::cout << "\nDisplaying gripper " << argv[4] << " at pose from " << argv[3] << " in PCD " << argv[2] << "\n\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (std::string(argv[2]), *cloud_ptr) == -1)
  {
    PCL_ERROR (("Couldn't read file " + std::string(argv[2]) + " \n").c_str());
    return (-1);
  }

  std::ifstream pose_file(argv[3]);
  if (!pose_file.is_open())
  {
    PCL_ERROR (("Couldn't read file " + std::string(argv[3]) + " \n").c_str());
    return (-1);
  }

  float x, y, z, qw, qx, qy, qz;
  pose_file >> x >> y >> z >> qw >> qx >> qy >> qz;

  Eigen::Affine3d world_to_grasp = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  if (display_pose)
  {
    viewer = visualize_pose(cloud_ptr, world_to_grasp);
  }
  else if (display_gripper)
  {
    pcl::PolygonMesh::Ptr gripper_ptr (new pcl::PolygonMesh);
    if (pcl::io::loadPLYFile(std::string(argv[4]), *gripper_ptr) == -1)
    {
      PCL_ERROR (("Couldn't read file " + std::string(argv[4]) + " \n").c_str());
      return (-1);
    }
    viewer = visualize_gripper(cloud_ptr, world_to_grasp, gripper_ptr);
  }

  viewer->setCameraPosition(-0.5,0,0,0,0,0,0,1,0);
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
