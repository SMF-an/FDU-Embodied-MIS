#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/movel.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class DualPoseMove final : public rclcpp::Node
{
public:
  DualPoseMove()
  : Node("rm_65_dual_pose")
  {
    const auto motion = declare_parameter<std::string>("motion", "movej_p");
    const auto left_pose = declare_parameter<std::vector<double>>(
      "left_pose", std::vector<double>{});
    const auto right_pose = declare_parameter<std::vector<double>>(
      "right_pose", std::vector<double>{});
    const auto speed = declare_parameter<int>("speed", 10);

    if (motion != "movej_p" && motion != "movel") {
      throw std::runtime_error("motion must be movej_p or movel.");
    }
    if (left_pose.size() != 7 || right_pose.size() != 7) {
      throw std::runtime_error(
        "Set left_pose and right_pose as [x,y,z,qx,qy,qz,qw] (meters and quaternion).");
    }
    if (speed < 1 || speed > 100) {
      throw std::runtime_error("speed must be between 1 and 100.");
    }

    left_target_ = make_pose(left_pose);
    right_target_ = make_pose(right_pose);
    left_result_ = create_subscription<std_msgs::msg::Bool>(
      result_topic("left_arm", motion), rclcpp::ParametersQoS(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        left_received_ = true;
        left_success_ = msg->data;
        RCLCPP_INFO(get_logger(), "Left pose motion result: %s", msg->data ? "success" : "failure");
      });
    right_result_ = create_subscription<std_msgs::msg::Bool>(
      result_topic("right_arm", motion), rclcpp::ParametersQoS(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        right_received_ = true;
        right_success_ = msg->data;
        RCLCPP_INFO(get_logger(), "Right pose motion result: %s", msg->data ? "success" : "failure");
      });

    if (motion == "movej_p") {
      left_movej_p_ = create_publisher<rm_ros_interfaces::msg::Movejp>(
        "/left_arm/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
      right_movej_p_ = create_publisher<rm_ros_interfaces::msg::Movejp>(
        "/right_arm/rm_driver/movej_p_cmd", rclcpp::ParametersQoS());
      wait_for_drivers(
        [this]() {return left_movej_p_->get_subscription_count();},
        [this]() {return right_movej_p_->get_subscription_count();});
      auto left_command = make_movej_p(left_target_, speed);
      auto right_command = make_movej_p(right_target_, speed);
      left_movej_p_->publish(left_command);
      right_movej_p_->publish(right_command);
    } else {
      left_movel_ = create_publisher<rm_ros_interfaces::msg::Movel>(
        "/left_arm/rm_driver/movel_cmd", rclcpp::ParametersQoS());
      right_movel_ = create_publisher<rm_ros_interfaces::msg::Movel>(
        "/right_arm/rm_driver/movel_cmd", rclcpp::ParametersQoS());
      wait_for_drivers(
        [this]() {return left_movel_->get_subscription_count();},
        [this]() {return right_movel_->get_subscription_count();});
      auto left_command = make_movel(left_target_, speed);
      auto right_command = make_movel(right_target_, speed);
      left_movel_->publish(left_command);
      right_movel_->publish(right_command);
    }
    RCLCPP_INFO(get_logger(), "Published both pose targets (%s).", motion.c_str());
  }

  void wait_for_results()
  {
    const auto deadline = std::chrono::steady_clock::now() + 120s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
      (!left_received_ || !right_received_))
    {
      rclcpp::spin_some(get_node_base_interface());
      rclcpp::sleep_for(20ms);
    }
    if (!left_received_ || !right_received_) {
      throw std::runtime_error("Timed out waiting for pose motion results; inspect both driver logs and arm states.");
    }
    if (!left_success_ || !right_success_) {
      throw std::runtime_error("Pose motion failed on at least one arm; inspect the corresponding driver logs.");
    }
  }

private:
  static std::string result_topic(const std::string & arm, const std::string & motion)
  {
    return "/" + arm + "/rm_driver/" + motion + "_result";
  }

  static geometry_msgs::msg::Pose make_pose(const std::vector<double> & values)
  {
    for (const auto value : values) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("Pose values must be finite.");
      }
    }
    const double q_norm = std::sqrt(
      values[3] * values[3] + values[4] * values[4] +
      values[5] * values[5] + values[6] * values[6]);
    if (q_norm < 1e-6) {
      throw std::runtime_error("Pose quaternion must have a non-zero norm.");
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = values[0];
    pose.position.y = values[1];
    pose.position.z = values[2];
    pose.orientation.x = values[3] / q_norm;
    pose.orientation.y = values[4] / q_norm;
    pose.orientation.z = values[5] / q_norm;
    pose.orientation.w = values[6] / q_norm;
    return pose;
  }

  static rm_ros_interfaces::msg::Movejp make_movej_p(
    const geometry_msgs::msg::Pose & pose, int speed)
  {
    rm_ros_interfaces::msg::Movejp command;
    command.pose = pose;
    command.speed = static_cast<uint8_t>(speed);
    command.trajectory_connect = 0;
    command.block = true;
    return command;
  }

  static rm_ros_interfaces::msg::Movel make_movel(
    const geometry_msgs::msg::Pose & pose, int speed)
  {
    rm_ros_interfaces::msg::Movel command;
    command.pose = pose;
    command.speed = static_cast<uint8_t>(speed);
    command.trajectory_connect = 0;
    command.block = true;
    return command;
  }

  template<typename LeftCount, typename RightCount>
  void wait_for_drivers(LeftCount left_count, RightCount right_count)
  {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
      (left_count() == 0 || right_count() == 0))
    {
      rclcpp::sleep_for(100ms);
    }
    if (left_count() == 0 || right_count() == 0) {
      throw std::runtime_error("Timed out waiting for both arm drivers to subscribe.");
    }
  }

  bool left_received_ = false;
  bool right_received_ = false;
  bool left_success_ = false;
  bool right_success_ = false;
  geometry_msgs::msg::Pose left_target_;
  geometry_msgs::msg::Pose right_target_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_result_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_result_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr left_movej_p_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr right_movej_p_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr left_movel_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr right_movel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DualPoseMove>();
    node->wait_for_results();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("rm_65_dual_pose"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
