//reference:An introduction to inertial navigation 6.1-6.2(https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf)
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "Eigen/Dense"
#include "math.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
using namespace Eigen;
sensor_msgs::Imu imu_data;
Matrix3f B,C,I,G;
Vector3f a_g,v_g,s_g;

geometry_msgs::Point pose_data;
visualization_msgs::Marker points, line_strip, line_list;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
  imu_data = *msg;
}

void imu_estimate(sensor_msgs::Imu& imu,geometry_msgs::Point *pose_data)
{
  float dt = 0.005;
  float sigma;
  Vector3f g_g(-0.007,0.308,9.69564);
  Vector3f a_b(imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z);
  float first(0), second(0);
/*  g_g(0,0) = 0;
  g_g(1,0) = 0;
  g_g(2,0) = 0;*/

/*  a_b(0,0) = imu.linear_acceleration.x;
  a_b(1,0) = imu.linear_acceleration.y;
  a_b(2,0) = imu.linear_acceleration.z;*/

  B(0,0) = 0;
  B(1,0) = imu.angular_velocity.z*dt;
  B(2,0) = -imu.angular_velocity.y*dt;
  B(0,1) = -imu.angular_velocity.z*dt;
  B(1,1) = 0;
  B(2,1) = imu.angular_velocity.x*dt;
  B(0,2) = imu.angular_velocity.y*dt;
  B(1,2) = -imu.angular_velocity.x*dt;
  B(2,2) = 0;

  sigma = sqrt( pow(imu.angular_velocity.x,2) + pow(imu.angular_velocity.y,2) + pow(imu.angular_velocity.z,2))*dt;

  //C = C*(I + (sin(sigma)/sigma)*B + ((1-cos(sigma))/pow(sigma,2))*B*B);
  //G = ((1-cos(sigma))/pow(sigma,2))*B*B;
  first = 1-pow(sigma,2)/2+pow(sigma,4)/120;
  second = 1/2-pow(sigma,2)/24+pow(sigma,4)/720;
  //ROS_INFO("first = %f , second = %f .",first, second);
  C = C*(I + first*B + second*B*B);

  a_g = C * a_b;

  /*compute pose*/
  v_g = v_g + dt*(a_g-g_g);
  s_g = s_g + dt*v_g;
  pose_data->x = s_g(0);
  pose_data->y = s_g(1);
  pose_data->z = 0;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw3");
  ros::NodeHandle nh;
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
                            ("/imu/data", 10, imu_cb);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  C.setZero();
  I.setZero();
  I(0,0) = 1;
  I(1,1) = 1;
  I(2,2) = 1;
  C = I;
  s_g.setZero();

  ros::Rate rate(200);
  float f = 0.0;
  while(ros::ok())
    {

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;



  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;



  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;



  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;


  imu_estimate(imu_data, &pose_data);



  points.points.push_back(pose_data);
  line_list.points.push_back(pose_data);
  line_list.points.push_back(pose_data);


  marker_pub.publish(points);
  marker_pub.publish(line_strip);
  marker_pub.publish(line_list);


 /* for (int row = 0; row < 3; ++row) {
    for (int column= 0; column < 3; ++column) {
      printf(" %f",C(row,column));
    }
    printf("\n");

  }*/
  ros::spinOnce();
  rate.sleep();

    }

return 0;


}
