#include <geranos_pole_trajectory/pole_trajectory_node.h>

namespace geranos_planner {

  PoleTrajectoryNode::PoleTrajectoryNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
     :  nh_(nh),
        private_nh_(private_nh) {
          ROS_INFO_STREAM("Constructor is called!");

          odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &PoleTrajectoryNode::odometryCallback, this);

          pole_transform_sub_ = nh_.subscribe("pole_transform", 1, &PoleTrajectoryNode::polePoseCallback, this);

          go_to_pole_service_ = nh_.advertiseService("go_to_pole_service", &PoleTrajectoryNode::goToPoleSrv, this);
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

  bool PoleTrajectoryNode::goToPoleSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    // Eigen::Vector3d current_position = current_position_W_;
    // Eigen::Vector3d pole_position = current_pole_position_W_;

    return true;
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

  std::vector<double> position1 = {0.0, 0.0, 0.5};
  std::vector<double> attitude1 = {0.0, 0.0, 0.0};

  std::vector<double> position2 = {0.0, 0.0, 1.5};
  std::vector<double> attitude2 = {0.0, 0.0, 0.0};

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

  yaml_point1["pos"] = position1;
  yaml_point1["att"] = attitude1;
  yaml_point1["stop"] = true;
  yaml_point1["time"] = 3.0;

  YAML::Node yaml_point2 = YAML::Node(YAML::NodeType::Map);

  yaml_point2["pos"] = position2;
  yaml_point2["att"] = attitude2;
  yaml_point2["stop"] = true;
  yaml_point2["time"] = 3.0;

  point_list.push_back(yaml_point1);
  point_list.push_back(yaml_point2);

  emitter << YAML::Value << point_list;
  emitter << YAML::EndMap;

  // write to file
  try
  {
    std::string filename = "/home/tim/catkin_ws/src/mav_ui/omav_local_planner/resource/yaml_test.yaml";
    std::ofstream fout;
    fout.open(filename.c_str());
    fout << emitter.c_str();
    fout.close();
    ROS_INFO_STREAM("SUCCESS");
  }
  catch (...)
  {
    ROS_INFO_STREAM("FAILED");
    return 0;
  }

  // ros::spin();

  return 0;
}
