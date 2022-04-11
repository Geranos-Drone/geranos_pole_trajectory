#include <geranos_pole_trajectory/pole_trajectory_node.h>

namespace geranos_planner {

  PoleTrajectoryNode::PoleTrajectoryNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
     :  nh_(nh),
        private_nh_(private_nh) {
          odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PoleTrajectoryNode::odometryCallback, this);
          pole_transform_sub_ = nh_.subscribe("pole_transform", 1, &PoleTrajectoryNode::polePoseCallback, this);
          go_to_pole_service_ = nh_.advertiseService("go_to_pole_service", &PoleTrajectoryNode::goToPoleSrv, this);
          go_to_pole_client_ = nh_.serviceClient<omav_local_planner::ExecuteTrajectory>("execute_trajectory");
          grab_pole_service_ = nh_.advertiseService("grab_pole_service", &PoleTrajectoryNode::grabPoleSrv, this);

          filename_go_to_pole_ = "/home/tim/catkin_ws/src/mav_ui/omav_local_planner/resource/go_to_pole.yaml";
          filename_grab_pole_ = "/home/tim/catkin_ws/src/mav_ui/omav_local_planner/resource/grab_pole.yaml";
          path_ = "/home/tim/catkin_ws/src/mav_ui/omav_local_planner/resource/";
        }

  PoleTrajectoryNode::~PoleTrajectoryNode() {}

  void PoleTrajectoryNode::odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("PoleTrajectoryNode received first odometry!");
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
    current_position_W_ = current_odometry_.position_W;
    current_orientation_W_B_ = current_odometry_.orientation_W_B;
  }

  void PoleTrajectoryNode::polePoseCallback(const geometry_msgs::TransformStamped& pole_transform_msg) {
    ROS_INFO_ONCE("Received first transform of Pole!");
    mav_msgs::eigenTrajectoryPointFromTransformMsg(pole_transform_msg, &pole_trajectory_point_);
    current_pole_position_W_ = pole_trajectory_point_.position_W;
  }

  bool PoleTrajectoryNode::writeYamlFile(const YAML::Emitter& emitter, const std::string& mode) {
    ROS_INFO_STREAM("writeYamlFile, mode = " << mode);
    std::string filename = path_ + mode + ".yaml";
    try
    {
      std::ofstream fout;
      fout.open(filename.c_str());
/*      if (mode == " go_to_pole")
        fout.open(filename_go_to_pole_.c_str());
      else if (mode == "grab_pole")
        fout.open(filename_grab_pole_.c_str());
      else {
        ROS_ERROR_STREAM("Could not write yaml file, wrong mode!");
        return false;
      }*/
      fout << emitter.c_str();
      fout.close();
      return true;
    }
    catch (...)
    {
      ROS_ERROR_STREAM("FAILED to write Yaml File!");
      return false;
    }
  }

  bool PoleTrajectoryNode::getTrajectoryToPole(const std::vector<double> &current_position, 
                                                const std::vector<double>& current_attitude,
                                                const std::vector<double> &pole_position,
                                                const std::string& mode) {

    std::vector<double> attitude = { 0.0, 0.0, current_attitude[2] };
    std::vector<double> position2;
    std::vector<double> position3;

    if (mode == "go_to_pole") {
      position2 = { current_position[0], current_position[1], pole_position[2] + 2.0 };
      position3 = { pole_position[0], pole_position[1], pole_position[2] + 2.0 };
    }
    else if (mode == "grab_pole") {
      position2 = { pole_position[0], pole_position[1], pole_position[2] + 2.0};
      position3 = { pole_position[0], pole_position[1], pole_position[2] };
    }
    else {
      ROS_ERROR_STREAM("Wrong Mode, could not get Trajectory!");
      return false;
    }

    YAML::Emitter emitter;

    emitter << YAML::BeginMap;
    emitter << YAML::Key << "order_rpy";
    emitter << YAML::Value << 1;
    emitter << YAML::Key << "forces";
    emitter << YAML::Value << false;
    emitter << YAML::Key << "torques";
    emitter << YAML::Value << false;
    emitter << YAML::Key << "times";
    emitter << YAML::Value << true;
    emitter << YAML::Key << "velocity_constraints";
    emitter << YAML::Value << true;
    emitter << YAML::Key << "points";

    YAML::Node point_list = YAML::Node(YAML::NodeType::Sequence);

    YAML::Node yaml_point1 = YAML::Node(YAML::NodeType::Map);

    yaml_point1["pos"] = current_position;
    yaml_point1["att"] = attitude;
    yaml_point1["stop"] = true;
    yaml_point1["time"] = 10.0;

    YAML::Node yaml_point2 = YAML::Node(YAML::NodeType::Map);

    yaml_point2["pos"] = position2;
    yaml_point2["att"] = attitude;
    yaml_point2["stop"] = true;
    yaml_point2["time"] = 10.0;

    YAML::Node yaml_point3 = YAML::Node(YAML::NodeType::Map);

    yaml_point3["pos"] = position3;
    yaml_point3["att"] = attitude;
    yaml_point3["stop"] = true;
    yaml_point3["time"] = 10.0;

    point_list.push_back(yaml_point1);
    point_list.push_back(yaml_point2);
    point_list.push_back(yaml_point3);

    emitter << YAML::Value << point_list;
    emitter << YAML::EndMap;

    return writeYamlFile(emitter, mode);
  }

  bool PoleTrajectoryNode::goToPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

    Eigen::Vector3d current_position = current_position_W_;
    Eigen::Vector3d pole_position = current_pole_position_W_;
    Eigen::Vector3d current_attitude; 
    mav_msgs::getEulerAnglesFromQuaternion(current_orientation_W_B_, &current_attitude);

    std::vector<double> current_position_vec = get_vec(current_position);
    std::vector<double> current_attitude_vec = get_vec(current_attitude);
    std::vector<double> pole_position_vec = get_vec(pole_position);

    if (getTrajectoryToPole(current_position_vec, current_attitude_vec, pole_position_vec, "go_to_pole")) {
      omav_local_planner::ExecuteTrajectory srv;
      srv.request.waypoint_filename = filename_go_to_pole_;
      if (go_to_pole_client_.call(srv))
        return true;
      else {
        ROS_ERROR_STREAM("Was not able to call execute_trajectory service!");
        return false;
      }
    }
    else {
      ROS_ERROR_STREAM("Was not able to get Trajectory to Pole!");
      return false;
    }
  }

  bool PoleTrajectoryNode::grabPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {

    Eigen::Vector3d current_position = current_position_W_;
    Eigen::Vector3d pole_position = current_pole_position_W_;
    Eigen::Vector3d current_attitude; 
    mav_msgs::getEulerAnglesFromQuaternion(current_orientation_W_B_, &current_attitude);

    std::vector<double> current_position_vec = get_vec(current_position);
    std::vector<double> current_attitude_vec = get_vec(current_attitude);
    std::vector<double> pole_position_vec = get_vec(pole_position);

    if (getTrajectoryToPole(current_position_vec, current_attitude_vec, pole_position_vec, "grab_pole")) {
      omav_local_planner::ExecuteTrajectory srv;
      srv.request.waypoint_filename = filename_grab_pole_;
    if (go_to_pole_client_.call(srv))
      return true;
    else {
      ROS_ERROR_STREAM("Was not able to call execute_trajectory service!");
      return false;
    }
    }
    else {
      ROS_ERROR_STREAM("Was not able to get Trajectory to Pole!");
      return false;
    }
  }

  template <typename eigen_vec>
  std::vector<double> PoleTrajectoryNode::get_vec(eigen_vec& vec) {
    std::vector<double> result(&vec[0], vec.data()+vec.size());
    return result;
  }
} //namespace geranos_planner

template<typename... Ts>
std::shared_ptr<geranos_planner::PoleTrajectoryNode> makeNode(Ts&&... params) {
  std::shared_ptr<geranos_planner::PoleTrajectoryNode> Node(new geranos_planner::PoleTrajectoryNode(std::forward<Ts>(params)...));
  return Node;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pole_trajectory_node");

  ros::NodeHandle nh, private_nh("~");

  auto Node = makeNode(nh, private_nh);

  ros::spin();

  return 0;
}
