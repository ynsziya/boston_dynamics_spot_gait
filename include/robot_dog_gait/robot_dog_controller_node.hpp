#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_dog_gait/body_pose_controller.hpp"
#include "robot_dog_gait/gait_engine.hpp"
#include "robot_dog_gait/leg_kinematics.hpp"
#include "robot_dog_gait/robot_dog_model.hpp"
#include "robot_dog_gait/trajectory_generator.hpp"

namespace robot_dog_gait
{

/// Orchestrates gait → trajectory → IK → joint commands.
class RobotDogControllerNode : public rclcpp::Node
{
public:
  explicit RobotDogControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadParameters();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void specialCmdCallback(const std_msgs::msg::String::SharedPtr msg);
  void controlTick();

  RobotDogModel model_;
  GaitParams gait_params_;
  GaitEngine gait_engine_;
  std::unique_ptr<TrajectoryGenerator> traj_;
  std::unique_ptr<BodyPoseController> pose_ctrl_;
  std::array<std::unique_ptr<LegKinematics>, 4> ik_;

  Twist2D cmd_{};
  BodyPose body_target_{};
  bool enabled_{true};

  double control_rate_hz_{100.0};
  std::string command_topic_;
  std::vector<std::string> joint_names_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr special_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace robot_dog_gait
