#ifndef GERANOS_POLE_TRAJECTORY_POLE_TRAJECTORY_NODE_H
#define GERANOS_POLE_TRAJECTORY_POLE_TRAJECTORY_NODE_H

#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_srvs/Empty.h>

#include <Eigen/Eigen>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

#include <iostream>
#include <fstream>
#include <string>
 


namespace geranos_planner {
  
  class PoleTrajectoryNode {

  public:
  	PoleTrajectoryNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  	~PoleTrajectoryNode();

  private:
  	void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  	void polePoseCallback(const geometry_msgs::TransformStamped pole_transform_msg);

  	bool goToPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);


  	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	ros::Subscriber odometry_sub_;
	ros::Subscriber pole_transform_sub_;

	ros::ServiceServer go_to_pole_service_;
	ros::ServiceServer grab_pole_service_;

	mav_msgs::EigenOdometry current_odometry_;
	Eigen::Vector3d current_position_W_;

	mav_msgs::EigenTrajectoryPoint pole_trajectory_point_;
	Eigen::Vector3d current_pole_position_W_;

  };
}

#endif