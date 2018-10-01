#include <cstdio>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#define _USE_MATH_DEFINES

geometry_msgs::PoseStamped pose;
nav_msgs::Path path;
geometry_msgs::PointStamped point;
nav_msgs::Odometry odometry;
geometry_msgs::Pose2D Pose2D;

geometry_msgs::PoseStamped pose_lsm;    //lsm=laser_scan_matcher
nav_msgs::Path path_lsm;                //lsm=laser_scan_matcher
geometry_msgs::PointStamped point_lsm;  //lsm=laser_scan_matcher

bool odom_receive=false;
bool map_receive=false;
bool receive=false;

void tf_callback(const tf::tfMessageConstPtr& tf)
{
 std::basic_string <char> odom_frame_id="odom";
 std::basic_string <char> map_frame_id="map";

 if(odom_frame_id==tf->transforms[0].header.frame_id)
 {
   odom_receive=true;
 }
 if(map_frame_id==tf->transforms[0].header.frame_id)
 {
   map_receive=true;
 }
 if(map_receive && odom_receive)
 {
   receive=true;
 }
}

void pose_callback(const geometry_msgs::Pose2DConstPtr& pose2D)
{

  float pose2D_x=pose2D->x;
  float pose2D_y=pose2D->y;
  float pose2D_theta=pose2D->theta;

  pose_lsm.pose.position.x=pose2D_x;
  pose_lsm.pose.position.y=pose2D_y;
  pose_lsm.pose.position.z=0;
  pose_lsm.pose.orientation.x=0;
  pose_lsm.pose.orientation.y=0;
  pose_lsm.pose.orientation.z=sin(pose2D_theta/2);
  pose_lsm.pose.orientation.w=cos(pose2D_theta/2);
  pose_lsm.header.frame_id="map";
  pose_lsm.header.stamp=ros::Time::now();
  path_lsm.poses.push_back(pose_lsm);
  path_lsm.header.frame_id="map";
  path_lsm.header.stamp=ros::Time::now();

  point_lsm.point.x=pose2D_x;
  point_lsm.point.y=pose2D_y;
  point_lsm.point.z=0;
  point_lsm.header.frame_id="map";
  point_lsm.header.stamp=ros::Time::now();

}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "position_publish", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::Publisher path_publisher=nh.advertise<nav_msgs::Path>("/gmapping/path",10);
  ros::Publisher point_publisher=nh.advertise<geometry_msgs::PointStamped>("/gmapping/pointstamped",10);
  ros::Publisher odometry_publisher=nh.advertise<nav_msgs::Odometry>("/gmapping/odometry",10);
  ros::Publisher Pose2D_publisher=nh.advertise<geometry_msgs::Pose2D>("/gmapping/Pose2D",10);
  ros::Publisher point_lsm_publisher=nh.advertise<geometry_msgs::PointStamped>("/laser_scan_matcher/point",10);
  ros::Publisher path_lsm_publisher=nh.advertise<nav_msgs::Path>("/laser_scan_matcher/path",10);
  ros::Subscriber tf_subscriber=nh.subscribe("/tf",10,tf_callback);
  ros::Subscriber pose2D_subscriber=nh.subscribe( "/pose2D" , 10 , pose_callback);

  double rate_hz=1;
  std::string source_frameid="map";
  std::string target_frameid="laser";
  ros::Rate rate(rate_hz);

  //Instantiate a local listener
  tf::TransformListener tf_listener;

  // Wait for up to one second for the first transforms to become avaiable.
  // echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
  tf_listener.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));
  //Nothing needs to be done except wait for a quit
  //The callbacks withing the listener class
  //will take care of everything
  while(nh.ok())
    {
        if(receive)
        {
            tf::StampedTransform echo_transform;
            tf_listener.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
            double yaw, pitch, roll;
            echo_transform.getBasis().getRPY(roll, pitch, yaw);
            tf::Vector3 transform = echo_transform.getOrigin();
            tf::Quaternion rotation= echo_transform.getRotation();

            Pose2D.x=transform.getX();
            Pose2D.y=transform.getY();
            Pose2D.theta=yaw*180.0/M_PI;;

            point.point.x=transform.getX();
            point.point.y=transform.getY();
            point.point.z=0;
            point.header.frame_id="map";
            point.header.stamp=ros::Time::now();

            odometry.pose.pose.position.x=transform.getX();
            odometry.pose.pose.position.y=transform.getY();
            odometry.pose.pose.position.z=0;
            odometry.pose.pose.orientation.x=0;
            odometry.pose.pose.orientation.y=0;
            odometry.pose.pose.orientation.z=rotation.getZ();
            odometry.pose.pose.orientation.w=rotation.getW();
            odometry.header.frame_id="map";
            odometry.header.stamp=ros::Time::now();

            pose.pose.position.x=transform.getX();
            pose.pose.position.y=transform.getY();
            pose.pose.position.z=0;
            pose.pose.orientation.x=0;
            pose.pose.orientation.y=0;
            pose.pose.orientation.z=rotation.getZ();
            pose.pose.orientation.w=rotation.getW();
            pose.header.frame_id="map";
            pose.header.stamp=ros::Time::now();
            path.poses.push_back(pose);
            path.header.frame_id="map";
            path.header.stamp=ros::Time::now();

            Pose2D_publisher.publish(Pose2D);
            point_publisher.publish(point);
            odometry_publisher.publish(odometry);
            path_publisher.publish(path);

            odom_receive=false;
            map_receive=false;
            receive=false;
        }
        point_lsm_publisher.publish(point_lsm);
        path_lsm_publisher.publish(path_lsm);
        ros::spinOnce();
        rate.sleep();
    }
}
