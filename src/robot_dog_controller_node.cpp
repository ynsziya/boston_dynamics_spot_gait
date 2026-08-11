#include "robot_dog_gait/robot_dog_controller_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace robot_dog_gait
{

RobotDogControllerNode::RobotDogControllerNode(const rclcpp::NodeOptions & options)
: Node("robot_dog_controller", options),
  model_(RobotDogModel::fromDefaults()),
  gait_engine_()
{
  loadParameters();

  gait_engine_.setParams(gait_params_);
  traj_ = std::make_unique<TrajectoryGenerator>(model_, gait_params_);
  pose_ctrl_ = std::make_unique<BodyPoseController>(model_);
  pose_ctrl_->setTarget(body_target_);

  for (int i = 0; i < 4; ++i) {
    ik_[static_cast<size_t>(i)] =
      std::make_unique<LegKinematics>(model_.legs[static_cast<size_t>(i)]);
  }

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&RobotDogControllerNode::cmdVelCallback, this, std::placeholders::_1));

  special_sub_ = create_subscription<std_msgs::msg::String>(
    "robot_dog/special_cmd", 10,
    std::bind(&RobotDogControllerNode::specialCmdCallback, this, std::placeholders::_1));

  joint_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&RobotDogControllerNode::controlTick, this));

  RCLCPP_INFO(
    get_logger(),
    "robot_dog_controller ready (gait=%s, rate=%.1f Hz, cmd=%s)",
    GaitEngine::toString(gait_params_.type), control_rate_hz_, command_topic_.c_str());
}

void RobotDogControllerNode::loadParameters()
{
  declare_parameter<double>("control_rate_hz", 100.0);
  declare_parameter<std::string>("command_topic", "/leg_position_controller/commands");
  declare_parameter<std::string>("gait.type", "trot");
  declare_parameter<double>("gait.frequency", 1.5);
  declare_parameter<double>("gait.duty_factor", 0.5);
  declare_parameter<double>("gait.swing_height", 0.06);
  declare_parameter<double>("gait.stance_depth", 0.0);
  declare_parameter<double>("gait.step_length", 0.12);
  declare_parameter<double>("gait.step_width", 0.06);
  declare_parameter<double>("body.nominal_height", model_.nominal_height);
  declare_parameter<double>("body.roll", 0.0);
  declare_parameter<double>("body.pitch", 0.0);

  control_rate_hz_ = get_parameter("control_rate_hz").as_double();
  command_topic_ = get_parameter("command_topic").as_string();

  gait_params_.type = GaitEngine::gaitFromString(get_parameter("gait.type").as_string());
  gait_params_.frequency = get_parameter("gait.frequency").as_double();
  gait_params_.duty_factor = get_parameter("gait.duty_factor").as_double();
  gait_params_.swing_height = get_parameter("gait.swing_height").as_double();
  gait_params_.stance_depth = get_parameter("gait.stance_depth").as_double();
  gait_params_.step_length = get_parameter("gait.step_length").as_double();
  gait_params_.step_width = get_parameter("gait.step_width").as_double();

  model_.nominal_height = get_parameter("body.nominal_height").as_double();
  body_target_.height = model_.nominal_height;
  body_target_.roll = get_parameter("body.roll").as_double();
  body_target_.pitch = get_parameter("body.pitch").as_double();

  joint_names_ = {
    "fl_hip_roll_joint", "fl_hip_yaw_joint", "fl_knee_joint",
    "fr_hip_roll_joint", "fr_hip_yaw_joint", "fr_knee_joint",
    "rl_hip_roll_joint", "rl_hip_yaw_joint", "rl_knee_joint",
    "rr_hip_roll_joint", "rr_hip_yaw_joint", "rr_knee_joint",
  };
}

void RobotDogControllerNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_.vx = msg->linear.x;
  cmd_.vy = msg->linear.y;
  cmd_.wz = msg->angular.z;
}

void RobotDogControllerNode::specialCmdCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & c = msg->data;
  if (c == "stand") {
    enabled_ = true;
    cmd_ = {};
    RCLCPP_INFO(get_logger(), "Stand / enable gait");
  } else if (c == "sit" || c == "stop") {
    enabled_ = false;
    cmd_ = {};
    RCLCPP_INFO(get_logger(), "Sit / disable gait");
  } else if (c == "trot" || c == "walk" || c == "pace" || c == "bound") {
    gait_params_.type = GaitEngine::gaitFromString(c);
    gait_engine_.setGait(gait_params_.type);
    traj_->setGaitParams(gait_params_);
    RCLCPP_INFO(get_logger(), "Gait -> %s", c.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Unknown special_cmd: %s", c.c_str());
  }
}

void RobotDogControllerNode::controlTick()
{
  const double dt = 1.0 / control_rate_hz_;
  std_msgs::msg::Float64MultiArray out;
  out.data.assign(12, 0.0);

  if (!enabled_) {
    // Hold a crouched default; refine later with sit trajectory.
    const std::array<double, 3> sit{0.0, 0.9, -1.8};
    for (int leg = 0; leg < 4; ++leg) {
      out.data[static_cast<size_t>(leg * 3 + 0)] = sit[0];
      out.data[static_cast<size_t>(leg * 3 + 1)] = sit[1];
      out.data[static_cast<size_t>(leg * 3 + 2)] = sit[2];
    }
    joint_pub_->publish(out);
    return;
  }

  const auto phases = gait_engine_.update(dt);
  std::array<bool, 4> in_stance{};
  for (int i = 0; i < 4; ++i) {
    in_stance[static_cast<size_t>(i)] = !phases[static_cast<size_t>(i)].in_swing;
  }

  BodyPose measured{};
  measured.height = model_.nominal_height;
  const auto pose_off = pose_ctrl_->footOffsets(measured, in_stance);

  for (int i = 0; i < 4; ++i) {
    const LegId id = static_cast<LegId>(i);
    auto ft = traj_->compute(id, phases[static_cast<size_t>(i)], cmd_);
    ft.position.x += pose_off[static_cast<size_t>(i)].x;
    ft.position.y += pose_off[static_cast<size_t>(i)].y;
    ft.position.z += pose_off[static_cast<size_t>(i)].z;

    JointAngles q;
    if (!ik_[static_cast<size_t>(i)]->inverseKinematics(ft.position, q)) {
      // Keep previous / zero on IK failure.
      continue;
    }
    out.data[static_cast<size_t>(i * 3 + 0)] = q.hip_roll;
    out.data[static_cast<size_t>(i * 3 + 1)] = q.hip_yaw;
    out.data[static_cast<size_t>(i * 3 + 2)] = q.knee;
  }

  joint_pub_->publish(out);
}

}  // namespace robot_dog_gait

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_dog_gait::RobotDogControllerNode>());
  rclcpp::shutdown();
  return 0;
}
