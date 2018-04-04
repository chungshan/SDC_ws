#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ICPbyPCL");
  ros::NodeHandle nh;
  /* ICP */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ>("map.pcd", *cloud1);
  pcl::io::loadPCDFile<pcl::PointXYZ>("scene.pcd", *cloud2);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
  tree1->setInputCloud(cloud2);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
  tree2->setInputCloud(cloud1);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setSearchMethodSource(tree1);
  icp.setSearchMethodTarget(tree2);
  icp.setMaxCorrespondenceDistance (1500);
  icp.setEuclideanFitnessEpsilon(0.1);
  icp.setTransformationEpsilon (1e-10);
  icp.setMaximumIterations (500);
  icp.setRANSACOutlierRejectionThreshold (1.5);
  icp.setInputSource(cloud2);
  icp.setInputTarget(cloud1);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  Eigen::Matrix4f R; //Transformation matrix
  R = icp.getFinalTransformation();

  ofstream myfile;
  myfile.open("Transformation matrix.txt");
  myfile << R;
  myfile.close();
  /*tf between frame map and scene*/




  tf::Vector3 origin;
  origin.setValue(static_cast<double>(R(0,3)),static_cast<double>(R(1,3)),static_cast<double>(R(2,3)));
  tf::Matrix3x3 tf3d;
  tf3d.setValue(static_cast<double>(R(0,0)),static_cast<double>(R(0,1)),static_cast<double>(R(0,2)),
                static_cast<double>(R(1,0)),static_cast<double>(R(1,1)),static_cast<double>(R(1,2)),
                static_cast<double>(R(2,0)),static_cast<double>(R(2,1)),static_cast<double>(R(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  tf::TransformBroadcaster br;


  /*Show result in rviz*/

  ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2>("map",1);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2>("scene",1);
  sensor_msgs::PointCloud2 output1, output2;
  pcl::toROSMsg(*cloud1, output1);
  pcl::toROSMsg(*cloud2, output2);

  output1.header.frame_id = std::string("map");
  output2.header.frame_id = std::string("scene");

  ros::Rate rate(50);
  while(ros::ok()){
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "scene"));
    pub1.publish(output1);
    pub2.publish(output2);

    rate.sleep();
  }
}
