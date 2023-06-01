#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class PathPlanning : public rclcpp::Node {
public:
  PathPlanning();

  void set_static_TF_frames();

private:
  // Params
  std::vector<double> front_left_buoy;
  std::vector<double> front_right_buoy;
  std::vector<double> back_left_buoy;
  std::vector<double> back_right_buoy;

  // Static TF broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf2_static_broadcaster;
};

// PathPlanning function definitions

// Constructor
PathPlanning::PathPlanning() : Node("path_planning_node") {
  RCLCPP_INFO(this->get_logger(), "Path planning node running");

  // Declare and get params
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Set coordinates of front left boja";
  this->declare_parameter<std::vector<double>>("front_left_buoy", {-50, 0},
                                               param_desc);
  param_desc.description = "Set coordinates of front right boja";
  this->declare_parameter<std::vector<double>>("front_right_buoy", {50, 0.0},
                                               param_desc);
  param_desc.description = "Set coordinates of back left boja";
  this->declare_parameter<std::vector<double>>("back_left_buoy", {-50, 150.0},
                                               param_desc);
  param_desc.description = "Set coordinates of back right boja";
  this->declare_parameter<std::vector<double>>("back_right_buoy", {50.0, 150.0},
                                               param_desc);

  this->tf2_static_broadcaster =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  this->set_static_TF_frames();
}

// Set static TF frames for corner bojas
void PathPlanning::set_static_TF_frames() {
  std::string child_frame_ids[4] = {"front_left_buoy", "front_right_buoy",
                                    "back_left_buoy", "back_right_buoy"};
  std::vector<double> param_arr[4] = {front_left_buoy, front_right_buoy,
                                      back_left_buoy, back_right_buoy};

  for (size_t i = 0; i < sizeof(child_frame_ids) / sizeof(std::string); i++) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_boat/odom";

    this->get_parameter<std::vector<double>>(child_frame_ids[i], param_arr[i]);

    RCLCPP_INFO(this->get_logger(), "Params: %.2f\n%.2f", param_arr[i][0],
                param_arr[i][1]);

    t.child_frame_id = child_frame_ids[i];

    t.transform.translation.x = param_arr[i][0];
    t.transform.translation.y = param_arr[i][1];
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    this->tf2_static_broadcaster->sendTransform(t);
  }
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanning>());
  rclcpp::shutdown();

  return 0;
}