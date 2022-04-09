#include <geranos_pole_trajectory/pole_trajectory_node.h>

namespace geranos_planner {

  PoleTrajectoryNode::PoleTrajectoryNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
     :  nh_(nh),
        private_nh_(private_nh) {
          ROS_INFO_STREAM("Constructor is called!");

          odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PoleTrajectoryNode::odometryCallback, this);

          pole_transform_sub_ = nh_.subscribe("pole_transform", 1, &PoleTrajectoryNode::polePoseCallback, this);

          go_to_pole_service_ = nh_.advertiseService("go_to_pole_service", &PoleTrajectoryNode::goToPoleSrv, this);

          go_to_pole_client_ = nh_.serviceClient<omav_local_planner::ExecuteTrajectory>("execute_trajectory");

          filename_ = "/home/tim/catkin_ws/src/mav_ui/omav_local_planner/resource/go_to_pole.yaml";
        }

  PoleTrajectoryNode::~PoleTrajectoryNode() {}

  void PoleTrajectoryNode::odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("PoleTrajectoryNode received first odometry!");
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
    current_position_W_ = current_odometry_.position_W;
  }

  void PoleTrajectoryNode::polePoseCallback(const geometry_msgs::TransformStamped pole_transform_msg) {
    ROS_INFO_ONCE("Received first transform of Pole!");
    mav_msgs::eigenTrajectoryPointFromTransformMsg(pole_transform_msg, &pole_trajectory_point_);
    current_pole_position_W_ = pole_trajectory_point_.position_W;
  }

  bool PoleTrajectoryNode::writeYamlFile(const YAML::Emitter& emitter) {
    try
    {
      std::ofstream fout;
      fout.open(filename_.c_str());
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

  bool PoleTrajectoryNode::getTrajectoryToPole(const std::vector<double> &current_position, const std::vector<double> &pole_position) {

    std::vector<double> attitude = {0.0, 0.0, 0.0};
    std::vector<double> position2 = {current_position[0], current_position[1], pole_position[2] + 2.0};
    std::vector<double> position3 = {pole_position[0], pole_position[1], pole_position[2] + 2.0};

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
    yaml_point1["time"] = 8.0;

    YAML::Node yaml_point2 = YAML::Node(YAML::NodeType::Map);

    yaml_point2["pos"] = position2;
    yaml_point2["att"] = attitude;
    yaml_point2["stop"] = true;
    yaml_point2["time"] = 8.0;

    YAML::Node yaml_point3 = YAML::Node(YAML::NodeType::Map);

    yaml_point3["pos"] = position3;
    yaml_point3["att"] = attitude;
    yaml_point3["stop"] = true;
    yaml_point3["time"] = 8.0;

    point_list.push_back(yaml_point1);
    point_list.push_back(yaml_point2);
    point_list.push_back(yaml_point3);

    emitter << YAML::Value << point_list;
    emitter << YAML::EndMap;

    return writeYamlFile(emitter);
  }

  bool PoleTrajectoryNode::goToPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    Eigen::Vector3d current_position = current_position_W_;
    Eigen::Vector3d pole_position = current_pole_position_W_;

    std::vector<double> current_position_vec(&current_position[0], 
                                              current_position.data()+current_position.cols()*current_position.rows());
    std::vector<double> pole_position_vec(&pole_position[0], 
                                              pole_position.data()+pole_position.cols()*pole_position.rows());

    if (getTrajectoryToPole(current_position_vec, pole_position_vec)) {
      omav_local_planner::ExecuteTrajectory srv;
      srv.request.waypoint_filename = filename_;
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

}

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
