#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>

// #include <pcl/point_types.h>
#include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/impl/kdtree.hpp>
#include <ros/package.h>
#include <vector>

typedef pcl::PointXYZ PointT;

using namespace std;
string file_name;

int add_boundary = 0;
int is_bridge = 0;
double downsample_res;
double map_offset_x, map_offset_y, map_offset_z;

int minus_twopointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud_input, pcl::PointCloud<pcl::PointXYZ> &cloud_input2, pcl::PointCloud<pcl::PointXYZ> &cloud_output)
{
  // cloud_input minus cloud_input2
  pcl::search::KdTree<pcl::PointXYZ> minus_kdtree;
  minus_kdtree.setInputCloud(cloud_input2.makeShared());

  vector<int> pointIdxRadiusSearch;
  vector<float> pointRadiusSquaredDistance;
  for (int i = 0; i < cloud_input.points.size(); i++)
  {
    if (minus_kdtree.radiusSearch(cloud_input.points[i], 0.6, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      continue;
    }
    cloud_output.points.push_back(cloud_input.points[i]);
  }

  ROS_INFO("AFTER MINUS, points count = %d", cloud_output.points.size());

  return cloud_output.points.size();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_recorder");
  ros::NodeHandle node("~");

  node.getParam("add_boundary", add_boundary);
  node.getParam("is_bridge", is_bridge);
  node.getParam("downsample_res", downsample_res);

  node.getParam("map_offset_x", map_offset_x);
  node.getParam("map_offset_y", map_offset_y);
  node.getParam("map_offset_z", map_offset_z);

  ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, true);
  ros::Publisher cloud_pub_no_boundary = node.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud_no_boundary", 10, true);
  file_name = argv[1];

  ros::Duration(1.0).sleep();

  /* load cloud from pcd */
  pcl::PointCloud<pcl::PointXYZ> cloud_temp, cloud;
  int status = pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud_temp);
  cloud = cloud_temp;
  //  cloud.clear();
  //  for (int i = 0; i < cloud_temp.size(); i+=5) {
  //      Eigen::Vector3d pt(cloud_temp.points[i].x, cloud_temp.points[i].y, cloud_temp.points[i].z);
  //      cloud.points.push_back(cloud_temp.points[i]);
  //  }

  if (status == -1)
  {
    cout << "can't read file." << endl;
    return -1;
  }

  // double x, y, w1, w2, h, _resolution = 0.1 ;
  //   x = 0.0;
  //   y = 0.0;
  //   w1 = 20.0;
  //   w2 = 30.0;

  //   x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  //   y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  //   int widNum1 = ceil(w1 / _resolution);
  //   int widNum2 = ceil(w2 / _resolution);
  //   pcl::PointXYZ pt_random;

  //   for (int r = -widNum1 / 2.0; r < widNum1 / 2.0; r++)
  //     for (int s = -widNum2 / 2.0; s < widNum2 / 2.0; s++) {
  //       h = 0.1;
  //       int heiNum = ceil(h / _resolution);
  //       for (int t = -0.5; t < heiNum; t++) {
  //         pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
  //         pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
  //         pt_random.z = (t + 0.5) * _resolution + 1e-2;
  //         cloud.points.push_back(pt_random);
  //       }
  //     }
  // string file_name2 = "/home/jackykong/motionplanning/FUEL_ws/src/FUEL/meshmap/hku_demo_color.ply";
  // pcl::PLYReader reader;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ply;
  // reader.read<pcl::PointXYZ>(file_name2,*cloud_ply);
  // pcl::io::savePCDFile("/home/jackykong/motionplanning/FUEL_ws/src/FUEL/meshmap/hku_demo_pcd.pcd",*cloud_ply);

  ROS_INFO("SUCCESS LOAD PCD FILE");

  // filter
  pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
  // _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  // _voxel_sampler.setInputCloud(cloud.makeShared());
  // _voxel_sampler.filter(cloud);

  // //remove outlier points, like humans
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloud.makeShared());
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (cloud);

  if (is_bridge == 1)
  {
    // Process map
    for (int i = 0; i < cloud.points.size(); ++i)
    {
      auto pt = cloud.points[i];
      pcl::PointXYZ pr;
      pr.x = pt.x;
      pr.y = -pt.z;
      pr.z = pt.y;
      cloud.points[i] = pr;
    }
  }

  // // Process map
  // for (int i = 0; i < cloud.points.size(); ++i)
  // {
  //   auto pt = cloud.points[i];
  //   pcl::PointXYZ pr;
  //   pr.x = pt.x - 40;
  //   pr.y = pt.y - 100;
  //   pr.z = pt.z - 80;
  //   cloud.points[i] = pr;
  // }

  pcl::PointXYZ global_mapmin;
  pcl::PointXYZ global_mapmax;

  pcl::getMinMax3D(cloud, global_mapmin, global_mapmax);

  ROS_WARN("Map bound: x=%f,%f, y=%f,%f, z=%f,%f", global_mapmin.x, global_mapmax.x, global_mapmin.y, global_mapmax.y, global_mapmin.z, global_mapmax.z);

  // Keep a copy without boundary for visualization
  pcl::PointCloud<pcl::PointXYZ> cloud_no_boundary = cloud;

  if (add_boundary == 1)
  {
    // for (double x = -10; x <= 10; x += 0.05)
    //   for (double y = -10; y <= 10; y += 0.05)
    //   {
    //     cloud.push_back(pcl::PointXYZ(x, y, 0));
    //   }

    // add boundary
    pcl::PointCloud<pcl::PointXYZ> cloud_boundary;
    int add_length = 1;
    int max_x = (int)global_mapmax.x + add_length;
    int min_x = (int)global_mapmin.x - add_length;
    int max_y = (int)global_mapmax.y + add_length;
    int min_y = (int)global_mapmin.y - add_length;
    int max_z = (int)global_mapmax.z + 6;
    int min_z = (int)global_mapmin.z + 0.5;
    int box_x = (max_x - min_x) * 10;
    int box_y = (max_y - min_y) * 10;
    int box_z = (max_z - min_z) * 10;
    ROS_WARN("Map bound: x=%d,%d, y=%d,%d, z=%d,%d", max_x, min_x, max_y, min_y, max_z, min_z);
    // draw six plane
    // draw 2 xy plane
    for (float i = min_x; i <= max_x; i = i + 0.1)
    {
      for (float j = min_y; j <= max_y; j = j + 0.1)
      {
        cloud.push_back(pcl::PointXYZ(i, j, min_z));
        cloud.push_back(pcl::PointXYZ(i, j, max_z));
      }
    }
    // draw 4 plane
    for (float k = min_z; k <= max_z; k = k + 0.1)
    {
      // draw x two lines
      for (float i = min_x; i <= max_x; i = i + 0.1)
      {
        cloud_boundary.push_back(pcl::PointXYZ(i, min_y, k));
        cloud_boundary.push_back(pcl::PointXYZ(i, max_y, k));
        cloud_boundary.push_back(pcl::PointXYZ(i, min_y - 0.1, k));
        cloud_boundary.push_back(pcl::PointXYZ(i, max_y + 0.1, k));
      }
      for (float j = min_y; j <= max_y; j = j + 0.1)
      {
        cloud_boundary.push_back(pcl::PointXYZ(min_x, j, k));
        cloud_boundary.push_back(pcl::PointXYZ(max_x, j, k));
        cloud_boundary.push_back(pcl::PointXYZ(min_x - 0.1, j, k));
        cloud_boundary.push_back(pcl::PointXYZ(max_x + 0.1, j, k));
      }
    }

    cloud_boundary.width = cloud_boundary.points.size();
    cloud_boundary.height = 1;

    cloud = cloud_boundary + cloud;

    ROS_INFO("ADD BOUNDARY!!!");
  }

  // pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
  _voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler.setInputCloud(cloud.makeShared());
  _voxel_sampler.filter(cloud);

  // Process map
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    auto pt = cloud.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x + map_offset_x;
    pr.y = pt.y + map_offset_y;
    pr.z = pt.z + map_offset_z;
    cloud.points[i] = pr;
  }

  // trans to mesh
  //  Resample the point cloud.
  //  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
  //  cloud_filtered = cloud.makeShared();
  //  pcl::search::KdTree<PointT>::Ptr treeSampling (new pcl::search::KdTree<PointT>);// KD-Tree for nearest-neighbor search.
  //  pcl::PointCloud<PointT> mls_point;    // MLS output.
  //  pcl::MovingLeastSquares<PointT,PointT> mls; // Moving least-squares object.
  //  mls.setComputeNormals(false);  // Whether to store computed normals during moving least-squares.
  //  mls.setInputCloud(cloud_filtered);         // Set the input cloud.
  //  mls.setPolynomialOrder(2);            // Fit a second-order polynomial.
  //  mls.setPolynomialFit(false);     // Set false to speed up smoothing.
  //  mls.setSearchMethod(treeSampling);         // Use KD-Tree as the search method.
  //  mls.setSearchRadius(0.05);           // Radius for fitting neighbors, in meters.
  //  mls.process(mls_point);                 // Output.

  //   // Output the resampling result.
  //   pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
  //   // cloud_smoothed = mls_point.makeShared();
  //   cloud_smoothed = cloud.makeShared();
  //   std::cout<<"cloud_smoothed: "<<cloud_smoothed->size() <<std::endl;
  //   //save cloud_smoothed
  //   // pcl::io::savePCDFileASCII("/tmp/cloud_smoothed.pcd",*cloud_smoothed);

  //   pcl::VoxelGrid<PointT> downSampled;  // Create the filtering object.
  //   downSampled.setInputCloud (cloud_smoothed);            // Set the input cloud to filter.
  //   downSampled.setLeafSize (0.01f, 0.01f, 0.01f);  // Use 1 cm cubic voxels for filtering.
  //   downSampled.filter (*cloud_smoothed);           // Run filtering and store the output.

  //   // Normal estimation.
  //   pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;             // Create the normal estimation object.
  //   normalEstimation.setInputCloud(cloud_smoothed);                         // Input point cloud.
  //   pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// KD-Tree for nearest-neighbor search.
  //   normalEstimation.setSearchMethod(tree);
  //   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // Output normal cloud.
  //   // K-neighbor mode can use k nearest points, or points inside a radius r; choose one.
  //   normalEstimation.setKSearch(20);// Use the nearest 10 points around the current point.
  //   //normalEstimation.setRadiusSearch(0.03);            // Use radius-neighbor search for each point.
  //   normalEstimation.compute(*normals);               // Compute normals.
  //   // Output normals.
  //   std::cout<<"normals: "<<normals->size()<<", "<<"normals fields: "<<pcl::getFieldsList(*normals)<<std::endl;
  //   // pcl::io::savePCDFileASCII("/tmp/normals.pcd",*normals);

  // 	// Concatenate point pose, color, and normal information.
  //   pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  //   pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
  //   // pcl::io::savePCDFileASCII("/tmp/cloud_with_normals.pcd",*cloud_with_normals);

  // // Greedy projection triangulation.
  //   // Define the search tree.
  //   pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  //   tree2->setInputCloud(cloud_with_normals);

  //   // // Triangulation.
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // Define the triangulation object.
  pcl::PolygonMesh triangles;                               // Store the final triangulated mesh.

  //   // Set triangulation parameters.
  //   gp3.setSearchRadius(0.1);  // Search radius, equivalent to the KNN sphere radius.
  //   gp3.setMu (2.5);  // Set the maximum nearest-neighbor distance to 2.5 times the sample spacing.
  //   gp3.setMaximumNearestNeighbors (100);    // Maximum searched neighbors; typical values are 50-100.

  //   gp3.setMinimumAngle(M_PI/18); // Minimum triangle angle after triangulation: 10 degrees.
  //   gp3.setMaximumAngle(2*M_PI/3); // Maximum triangle angle after triangulation: 120 degrees.

  //   gp3.setMaximumSurfaceAngle(M_PI/4); // Maximum normal deviation from the sample normal: 45 degrees.
  //   gp3.setNormalConsistency(false);  // Set true to enforce normal consistency; false skips the check.

  //   gp3.setInputCloud (cloud_with_normals);     // Set the input cloud to an oriented point cloud.
  //   gp3.setSearchMethod (tree2);   // Set the search method.
  //   gp3.reconstruct (triangles);  // Reconstruct and extract the triangulation.
  //   pcl::io::saveOBJFile("/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/uav_simulator/map_generator/resource/result.obj", triangles);
  //   std::cout<<" Out put finished"<<std::endl;

  // string objPath = "/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/uav_simulator/map_generator/resource/result.obj"; // OBJ file in the current directory.
  // // Read.
  // pcl::PolygonMesh mesh;
  // pcl::io::loadPolygonFileOBJ(objPath, triangles);

  // // Display the meshing result.
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  // viewer->setBackgroundColor(0, 0, 0);  //
  // viewer->addPolygonMesh(triangles, "wangge");  //

  // cout << "Publishing map..." << endl;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "world";
  ROS_INFO("Map point size = %d", cloud.points.size());

  // Apply same processing to cloud_no_boundary
  pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler_no_boundary;
  _voxel_sampler_no_boundary.setLeafSize(downsample_res, downsample_res, downsample_res);
  _voxel_sampler_no_boundary.setInputCloud(cloud_no_boundary.makeShared());
  _voxel_sampler_no_boundary.filter(cloud_no_boundary);

  for (int i = 0; i < cloud_no_boundary.points.size(); ++i)
  {
    auto pt = cloud_no_boundary.points[i];
    pcl::PointXYZ pr;
    pr.x = pt.x + map_offset_x;
    pr.y = pt.y + map_offset_y;
    pr.z = pt.z + map_offset_z;
    cloud_no_boundary.points[i] = pr;
  }

  sensor_msgs::PointCloud2 msg_no_boundary;
  pcl::toROSMsg(cloud_no_boundary, msg_no_boundary);
  msg_no_boundary.header.frame_id = "world";
  ROS_INFO("Map point size (no boundary) = %d", cloud_no_boundary.points.size());

  // write files
  //  std::string pkg_path("/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/uav_simulator/map_generator/resource");
  //  std::string pcd_name("watertre03_cutoff");
  //  if(pcl::io::savePCDFileASCII (pkg_path+pcd_name+".pcd", cloud)>=0)
  //  {std::cerr << "Saved  " << pcd_name<<".pcd"<< std::endl;}

  // ros::Publisher minus_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/minus_cloud", 10, true);
  // pcl::PointCloud<pcl::PointXYZ> cloud_minus, cloud_output;
  // status = pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jackykong/motionplanning/FUEL_ws/src/Exploration_sim/octomap_mapping/octomap_server/data/FUEL_WATERTRE03NEW2.pcd", cloud_minus);
  // if (status == -1)
  // {
  //   cout << "can't read file." << endl;
  //   return -1;
  // }
  // minus_twopointcloud(cloud,cloud_minus,cloud_output);
  // sensor_msgs::PointCloud2 minus_msg;
  // pcl::toROSMsg(cloud_output, minus_msg);
  // minus_msg.header.frame_id = "world";

  int count = 0;
  while (ros::ok()) //! viewer->wasStopped()
  {

    // viewer->spinOnce(100);
    // boost::this_thread::sleep(boost::posix_time::microseconds(100000));

    ros::Duration(1.0).sleep();
    cloud_pub.publish(msg);
    cloud_pub_no_boundary.publish(msg_no_boundary);
    // minus_cloud_pub.publish(minus_msg);
    ++count;
    if (count > 10)
    {
      break;
    }
  }
  cout << "finish publish map." << endl;

  return 0;
}
