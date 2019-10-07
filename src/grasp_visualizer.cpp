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
#include <string>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

// --------------
// -----Help-----  #TODO: switch to boost::program_options
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\nUsage: "<<progName<<" pcloud.pcd [pose.txt] [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h                    This help\n\n"
            << "-g gripper.ply        Display gripper in place of frame\n\n"
            << "-r roi.txt            Display transparent Region Of Interest box\n\n"
            << "-e excl.roi           Display exclusion boxes from file\n\n"
            << "pose.txt in format \"x y z qw qx qy qz\"\n"
            << "roi.txt  in format \"x y z qw qx qy qz Xdim Ydim Zdim\"\n"
            << "excl.roi in format \"x y z qw qx qy qz Xdim Ydim Zdim\"\n"
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
  if ( (pcl::console::find_argument (argc, argv, "-h") >= 0) || (argc < 2) )
  {
    printUsage (argv[0]);
    return 0;
  }

  bool display_gripper(false), display_roi(false), display_excl(false);

  int gripper_arg = pcl::console::find_argument (argc, argv, "-g");
  int roi_arg = pcl::console::find_argument (argc, argv, "-r");
  int excl_arg = pcl::console::find_argument (argc, argv, "-e");

  if ( (argc >= 3) && (argv[1][0] != '-') && (argv[2][0] != '-') )
  {
    std::cout << "\nDisplaying pose from " << argv[2] << " in PCD " << argv[1] << "\n\n";
  }
  else if ( (gripper_arg >= 0) && (argc > (gripper_arg + 1)) && (argv[gripper_arg+1][0] != '-') )
  {
    std::cout << "\nDisplaying gripper " << argv[gripper_arg+1] << " at pose from " << argv[2] << " in PCD " << argv[1] << "\n\n";
  }
  else if (argc == 2)
  {
    std::cout << "\nDisplaying pointcloud " << argv[1] << "\n\n";
  }
  else
  {
    printUsage (argv[0]);
    return 0;
  }

  if (roi_arg >= 0)
  {
    if ( (argc > (roi_arg + 1)) && (argv[roi_arg+1][0] != '-') )
    {
      std::cout << "\nDisplaying region of interest from " << argv[roi_arg+1] << "\n\n";
    }
    else
    {
      printUsage (argv[0]);
      return 0;
    }
  }

  if (excl_arg >= 0)
  {
    if ( (argc > (excl_arg + 1)) && (argv[excl_arg+1][0] != '-') )
    {
      std::cout << "\nDisplaying exclusion boxes from " << argv[excl_arg+1] << "\n\n";
    }
    else
    {
      printUsage (argv[0]);
      return 0;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (std::string(argv[1]), *cloud_ptr) == -1)
  {
    PCL_ERROR (("Couldn't read file " + std::string(argv[1]) + " \n").c_str());
    return (-1);
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  // If only provided a pcd then just display it
  if (argc == 2)
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_ptr, rgb, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(-0.5,0,0,0,0,0,0,1,0);
  
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
  }

  std::ifstream pose_file(argv[2]);
  if (!pose_file.is_open())
  {
    PCL_ERROR (("Couldn't read file " + std::string(argv[2]) + " \n").c_str());
    return (-1);
  }

  float x, y, z, qw, qx, qy, qz;
  pose_file >> x >> y >> z >> qw >> qx >> qy >> qz;

  Eigen::Affine3d world_to_grasp = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);

  if (roi_arg > 0)
  {
    std::ifstream roi_file(argv[roi_arg+1]);
    if (!roi_file.is_open())
    {
      PCL_ERROR (("Couldn't read file " + std::string(argv[roi_arg+1]) + " \n").c_str());
      return (-1);
    }

    float roi_x, roi_y, roi_z, roi_qw, roi_qx, roi_qy, roi_qz, xdim, ydim, zdim;
    roi_file >> roi_x >> roi_y >> roi_z >> roi_qw >> roi_qx >> roi_qy >> roi_qz >> xdim >> ydim >> zdim;

    viewer->addCube(world_to_grasp.inverse().cast <float> () * Eigen::Vector3f(roi_x,roi_y,roi_z), Eigen::Quaterniond(qw, qx, qy, qz).inverse().cast <float> () * Eigen::Quaternionf(roi_qw,roi_qx,roi_qy,roi_qz), xdim, ydim, zdim);
  }

  viewer->setRepresentationToWireframeForAllActors ();

  if (excl_arg > 0)
  {
    std::ifstream excl_file(argv[excl_arg+1]);
    if (!excl_file.is_open())
    {
      PCL_ERROR (("Couldn't read file " + std::string(argv[excl_arg+1]) + " \n").c_str());
      return (-1);
    }

    std::string line_in;
    int number = 0;
    while (std::getline(excl_file, line_in))
    {
      std::stringstream iss(line_in);

      float excl_x, excl_y, excl_z, excl_qw, excl_qx, excl_qy, excl_qz, xdim, ydim, zdim;
      iss >> excl_x >> excl_y >> excl_z >> excl_qw >> excl_qx >> excl_qy >> excl_qz >> xdim >> ydim >> zdim;

      std::ostringstream stm;
      stm << "excl_box_";
      stm << number++;
      viewer->addCube(Eigen::Vector3f(excl_x,excl_y,excl_z), Eigen::Quaternionf(excl_qw,excl_qx,excl_qy,excl_qz), xdim, ydim, zdim, stm.str());
    }

  }

  if (gripper_arg > 0)
  {
    pcl::PolygonMesh::Ptr gripper_ptr (new pcl::PolygonMesh);
    if (pcl::io::loadPLYFile(std::string(argv[gripper_arg+1]), *gripper_ptr) == -1)
    {
      PCL_ERROR (("Couldn't read file " + std::string(argv[gripper_arg+1]) + " \n").c_str());
      return (-1);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_ptr, *cloud_transformed, world_to_grasp.inverse());

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_transformed);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_transformed, rgb, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->initCameraParameters ();

    viewer->addPolygonMesh(*gripper_ptr);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_ptr, *cloud_transformed, world_to_grasp.inverse());

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_transformed);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_transformed, rgb, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addCoordinateSystem (0.2);
    viewer->initCameraParameters ();
  }

  viewer->setCameraPosition(-0.5,0,0,0,0,0,0,1,0);
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
