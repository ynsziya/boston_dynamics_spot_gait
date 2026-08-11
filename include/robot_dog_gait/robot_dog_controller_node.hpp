#ifndef ROBOT_DOG_GAIT__ROBOT_DOG_CONTROLLER_NODE_HPP_
#define ROBOT_DOG_GAIT__ROBOT_DOG_CONTROLLER_NODE_HPP_

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "robot_dog_gait/body_pose_controller.hpp"
#include "robot_dog_gait/gait_engine.hpp"
#include "robot_dog_gait/leg_kinematics.hpp"
#include "robot_dog_gait/robot_dog_model.hpp"
#include "robot_dog_gait/trajectory_generator.hpp"

namespace robot_dog_gait
{

/// Top-level ROS2 orchestrator. Wires together every pure-C++ layer:
///
///   cmd_vel (+ IMU) --> GaitEngine --> TrajectoryGenerator --> BodyPoseController
///                                                 |
///                                                 v
///                                          LegKinematics (IK)
///                                                 |
///                                                 v
///                              Float64MultiArray -> position controller
///
/// Runs a fixed-rate control loop (control_frequency_hz, default 100 Hz):
/// each tick it reads the latest teleop command (applying a deadband, a
/// speed clamp, and a "stale command" safety timeout), advances the gait
/// clock, computes each leg's stride from the commanded body velocity
/// (linear.x/y + the tangential contribution of angular.z at that leg's
/// mount point -- this is what makes turning look natural instead of all
/// four legs taking identical steps), solves IK, and publishes all 12
/// joint angles in one shot.
class RobotDogControllerNode : public rclcpp::Node
{
public:
  RobotDogControllerNode();

private:
  void declareParameters();
  void applyGaitTypeParam(const std::string & name);
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & params);

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void controlLoop();

  // --- ROS interfaces ------------------------------------------------
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // --- Kinematic / gait core (pure C++, unit-testable in isolation) -----
  RobotDogModel model_;
  std::array<LegKinematics, 4> leg_kinematics_;
  std::array<Vec3, 4> neutral_foot_position_;  ///< per-leg, leg-local frame (hip_roll origin)
  GaitEngine gait_;
  TrajectoryGenerator trajectory_;
  BodyPoseController body_pose_;

  // --- Teleop / sensor state (written from subscription callbacks, read
  // from the control-loop timer -- guarded so this stays safe even if a
  // multi-threaded executor is used later) -----------------------------
  std::mutex state_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  rclcpp::Time last_cmd_time_;
  bool have_cmd_{false};
  double latest_roll_{0.0};
  double latest_pitch_{0.0};
  bool have_imu_{false};

  rclcpp::Time last_loop_time_;
  bool have_last_loop_time_{false};

  // --- Cached parameters ------------------------------------------------
  double control_frequency_hz_{100.0};
  double cmd_vel_timeout_sec_{0.5};
  double linear_deadband_{0.01};
  double angular_deadband_{0.02};
  double max_linear_speed_{0.4};
  double max_angular_speed_{1.0};
  double min_step_frequency_hz_{0.8};
  double max_step_frequency_hz_{2.2};
  double max_stride_{0.18};
  bool use_imu_feedback_{false};
};

}  // namespace robot_dog_gait

#endif  // ROBOT_DOG_GAIT__ROBOT_DOG_CONTROLLER_NODE_HPP_
