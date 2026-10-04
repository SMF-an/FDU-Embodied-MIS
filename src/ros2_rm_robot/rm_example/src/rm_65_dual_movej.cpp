#include <chrono>
#include <cstdint>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class DualMoveJ final : public rclcpp::Node
{
public:
  DualMoveJ()
  : Node("rm_65_dual_movej")
  {
    const auto left_joints = declare_parameter<std::vector<double>>(
      "left_joints", std::vector<double>{});
    const auto right_joints = declare_parameter<std::vector<double>>(
      "right_joints", std::vector<double>{});
    const auto unit = declare_parameter<std::string>("joint_unit", "radians");
    if (unit != "radians" && unit != "degrees") {
      throw std::runtime_error("joint_unit must be radians or degrees.");
    }
    for (const auto & joints : {left_joints, right_joints}) {
      for (const auto value : joints) {
        if (!std::isfinite(value) ||
          (unit == "radians" && std::abs(value) > 6.283185307179586))
        {
          throw std::runtime_error("Invalid joint angle; for degree inputs set joint_unit:=degrees.");
        }
      }
    }
    const auto speed = declare_parameter<int>("speed", 10);

    if (left_joints.size() != 6 || right_joints.size() != 6) {
      throw std::runtime_error(
        "Set both left_joints and right_joints to six joint angles in radians.");
    }
    if (speed < 1 || speed > 100) {
      throw std::runtime_error("speed must be between 1 and 100.");
    }

    left_publisher_ = create_publisher<rm_ros_interfaces::msg::Movej>(
      "/left_arm/rm_driver/movej_cmd", rclcpp::ParametersQoS());
    right_publisher_ = create_publisher<rm_ros_interfaces::msg::Movej>(
      "/right_arm/rm_driver/movej_cmd", rclcpp::ParametersQoS());

    left_result_ = create_subscription<std_msgs::msg::Bool>(
      "/left_arm/rm_driver/movej_result", rclcpp::ParametersQoS(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        left_received_ = true;
        left_success_ = msg->data;
        RCLCPP_INFO(get_logger(), "Left MoveJ result: %s", msg->data ? "success" : "failure");
      });
    right_result_ = create_subscription<std_msgs::msg::Bool>(
      "/right_arm/rm_driver/movej_result", rclcpp::ParametersQoS(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        right_received_ = true;
        right_success_ = msg->data;
        RCLCPP_INFO(get_logger(), "Right MoveJ result: %s", msg->data ? "success" : "failure");
      });
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline &&
      (left_publisher_->get_subscription_count() == 0 ||
      right_publisher_->get_subscription_count() == 0))
    {
      rclcpp::sleep_for(100ms);
    }

    if (left_publisher_->get_subscription_count() == 0 ||
      right_publisher_->get_subscription_count() == 0)
    {
      throw std::runtime_error("Timed out waiting for both arm drivers. Launch rm_65_dual_bringup.launch.py and check /left_arm/rm_driver/movej_cmd and /right_arm/rm_driver/movej_cmd.");
    }

    left_publisher_->publish(make_command(left_joints, speed, unit));
    right_publisher_->publish(make_command(right_joints, speed, unit));
    RCLCPP_INFO(get_logger(), "Published targets (%s); waiting for driver results.", unit.c_str());
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
      throw std::runtime_error("Timed out waiting for MoveJ results; inspect driver logs and actual arm state before retrying.");
    }
    if (!left_success_ || !right_success_) {
      throw std::runtime_error("MoveJ failed; inspect the corresponding driver error code.");
    }
  }

private:
  static rm_ros_interfaces::msg::Movej make_command(
    const std::vector<double> & joints, int speed, const std::string & unit)
  {
    rm_ros_interfaces::msg::Movej command;
    for (const auto value : joints) {
      command.joint.push_back(static_cast<float>(
        unit == "degrees" ? value * 0.017453292519943295 : value));
    }
    command.speed = static_cast<uint8_t>(speed);
    command.dof = 6;
    command.block = true;
    command.trajectory_connect = 0;
    return command;
  }

  bool left_received_ = false, right_received_ = false;
  bool left_success_ = false, right_success_ = false;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_result_, right_result_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr left_publisher_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr right_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DualMoveJ>();
    node->wait_for_results();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("rm_65_dual_movej"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
