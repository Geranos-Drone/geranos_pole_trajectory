#ifndef GERANOS_POLE_TRAJECTORY_POLE_TRAJECTORY_NODE_H
#define GERANOS_POLE_TRAJECTORY_POLE_TRAJECTORY_NODE_H

#include <ros/ros.h>

//msgs
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/common.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>

//tf
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

//srvs
#include <std_srvs/Empty.h>
#include <omav_local_planner/ExecuteTrajectory.h>

//eigen
#include <Eigen/Eigen>

//eigen
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/emitter.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


namespace geranos_planner {
  
  class PoleTrajectoryNode {

  public:
  	PoleTrajectoryNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  	~PoleTrajectoryNode();

  private:
  	void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  	void transformOdometry(mav_msgs::EigenOdometry& odometry);

  	void whitePolePoseCallback(const geometry_msgs::TransformStamped& pole_transform_msg);
  	void greyPolePoseCallback(const geometry_msgs::TransformStamped& pole_transform_msg);
  	void mountPoseCallback(const geometry_msgs::TransformStamped& pole_transform_msg);

  	bool writeYamlFile(const YAML::Emitter& emitter, const std::string& mode);

  	bool getTrajectoryToPole(const std::vector<double> &current_position,  
  		                                          const std::vector<double>& current_attitude,
                                                const std::vector<double> &pole_position,
                                                const std::string& mode);

  	bool goToPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  	bool grabPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  	bool resetSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  	void publishMode();

	  template<typename eigen_vec>
	  std::vector<double> get_vec(eigen_vec& vec);

  	ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		ros::Subscriber odometry_sub_;
		ros::Subscriber pole_white_transform_sub_;
		ros::Subscriber pole_grey_transform_sub_;
		ros::Subscriber mount_transform_sub_;
		ros::Publisher mode_pub_;

		ros::ServiceServer go_to_pole_service_;
		ros::ServiceServer grab_pole_service_;
		ros::ServiceServer reset_trajectory_service_;
		ros::ServiceClient go_to_pole_client_;

	  tf::TransformListener tf_listener_;
	  tf::StampedTransform tf_imu_base_;
	  Eigen::Affine3d T_B_imu_;

		mav_msgs::EigenOdometry current_odometry_;
		Eigen::Vector3d current_position_W_;
		double current_yaw_W_B_;

		mav_msgs::EigenTrajectoryPoint pole_white_trajectory_point_;
		mav_msgs::EigenTrajectoryPoint pole_grey_trajectory_point_;
		mav_msgs::EigenTrajectoryPoint mount_trajectory_point_;
		Eigen::Vector3d current_pole_white_position_W_;
		Eigen::Vector3d current_pole_grey_position_W_;
		Eigen::Vector3d current_mount_position_W_;

		std::string filename_go_to_pole_;
		std::string filename_grab_pole_;
		std::string path_;

		std::string mode_;
		bool grabbed_white_;
		bool grabbed_grey_;

  };
}

#endif