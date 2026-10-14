#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <interactive_markers/interactive_marker_server.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/perception_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

class Rm65RcmDemo : public rclcpp::Node
{
public:
  explicit Rm65RcmDemo(const rclcpp::NodeOptions & options)
  : Node("rm65_rcm_demo", options)
  {
    const auto declare_if_missing = [this](const auto & name, const auto & value) {
        if (!has_parameter(name)) {
          declare_parameter(name, value);
        }
      };
    declare_if_missing("planning_group", std::string("rm_group"));
    declare_if_missing("current_state_wait_sec", 30.0);
    declare_if_missing("velocity_scaling", 0.08);
    declare_if_missing("acceleration_scaling", 0.08);
    declare_if_missing("planning_time", 10.0);
    declare_if_missing("derive_rcm_from_current", true);
    declare_if_missing("rcm_x", 0.0);
    declare_if_missing("rcm_y", 0.0);
    declare_if_missing("rcm_z", 0.0);
    declare_if_missing("rcm_distance_from_tip_m", 0.15);
    declare_if_missing("eef_step_m", 0.001);
    declare_if_missing("rcm_tolerance_m", 0.002);
    declare_if_missing("max_rcm_rotation_deg", 0.0);
    declare_if_missing("max_rcm_insertion_m", 0.0);
    declare_if_missing("obstacle_scene_update_threshold_m", 0.005);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "rcm_demo/markers", rclcpp::QoS(10));
    current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "rcm_demo/current_tool_pose", rclcpp::QoS(10));
    rcm_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "rcm_demo/rcm_point", rclcpp::QoS(10));
    motion_busy_pub_ = create_publisher<std_msgs::msg::Bool>(
      "rcm_demo/motion_busy", rclcpp::QoS(1).transient_local());
    web_target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "rcm_demo/web_target", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr target) {
        onWebTarget(target);
      });
    playback_start_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "rcm_demo/playback_start_target", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr target) {
        onPlaybackStartTarget(target);
      });
    playback_start_joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "rcm_demo/playback_start_joints", rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr target) {
        onPlaybackStartJoints(target);
      });
    playback_start_result_pub_ = create_publisher<std_msgs::msg::UInt64>(
      "rcm_demo/playback_start_result", rclcpp::QoS(10).transient_local());
    set_rcm_point_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto set_rcm_point_options = rclcpp::SubscriptionOptions();
    set_rcm_point_options.callback_group = set_rcm_point_callback_group_;
    set_rcm_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "rcm_demo/set_rcm_point", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr point) {
        onSetRcmPoint(point);
      }, set_rcm_point_options);
    set_rcm_result_pub_ = create_publisher<std_msgs::msg::Bool>(
      "rcm_demo/set_rcm_point_result", rclcpp::QoS(10));
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "rcm_demo/stop", rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr stop) {
        if (stop->data && move_group_) {
          move_group_->stop();
          RCLCPP_WARN(get_logger(), "收到 Web 遥控器停止命令");
        }
      });
    perception_scene_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto perception_scene_options = rclcpp::SubscriptionOptions();
    perception_scene_options.callback_group = perception_scene_callback_group_;
    perception_scene_sub_ = create_subscription<rm_ros_interfaces::msg::PerceptionScene>(
      "/perception/scene", rclcpp::QoS(1),
      [this](rm_ros_interfaces::msg::PerceptionScene::ConstSharedPtr scene) {
        onPerceptionScene(scene);
      }, perception_scene_options);
    collision_scene_synced_pub_ = create_publisher<std_msgs::msg::UInt64>(
      "rcm_demo/collision_scene_synced", rclcpp::QoS(1).transient_local());
  }

  void waitForInteractiveMotion()
  {
    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
  }

  void waitForStatePublisher()
  {
    if (state_thread_.joinable()) {
      state_thread_.join();
    }
  }

  bool init()
  {
    const std::string group = get_parameter("planning_group").as_string();
    const moveit::planning_interface::MoveGroupInterface::Options move_group_options(
      group,
      moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
      get_namespace());
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), move_group_options);
    move_group_->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    move_group_->setMaxAccelerationScalingFactor(
      get_parameter("acceleration_scaling").as_double());
    move_group_->setPlanningTime(get_parameter("planning_time").as_double());
    planning_scene_interface_ = std::make_unique<
      moveit::planning_interface::PlanningSceneInterface>(get_namespace());

    const double state_wait = get_parameter("current_state_wait_sec").as_double();
    if (!move_group_->startStateMonitor(state_wait)) {
      RCLCPP_ERROR(get_logger(), "MoveIt 未能启动机器人状态监视");
      return false;
    }
    if (!move_group_->getCurrentState(state_wait)) {
      RCLCPP_ERROR(get_logger(), "等待当前关节状态超时");
      return false;
    }

    end_effector_link_ = move_group_->getEndEffectorLink();
    if (end_effector_link_.empty()) {
      RCLCPP_ERROR(get_logger(), "规划组没有配置末端 link");
      return false;
    }
    RCLCPP_INFO(get_logger(), "RCM 规划组=%s, 尖端 link=%s, 参考系=%s",
      group.c_str(), end_effector_link_.c_str(), move_group_->getPlanningFrame().c_str());
    return true;
  }

  bool runDemo()
  {
    geometry_msgs::msg::PoseStamped start_stamped =
      move_group_->getCurrentPose(end_effector_link_);
    const Eigen::Vector3d start_tip(
      start_stamped.pose.position.x,
      start_stamped.pose.position.y,
      start_stamped.pose.position.z);
    Eigen::Quaterniond start_orientation(
      start_stamped.pose.orientation.w,
      start_stamped.pose.orientation.x,
      start_stamped.pose.orientation.y,
      start_stamped.pose.orientation.z);
    if (start_orientation.norm() < 1e-9) {
      RCLCPP_ERROR(get_logger(), "当前末端四元数无效");
      return false;
    }
    start_orientation.normalize();
    const Eigen::Vector3d shaft_axis = start_orientation * Eigen::Vector3d::UnitZ();

    const bool derive_rcm = get_parameter("derive_rcm_from_current").as_bool();
    const double rcm_distance = get_parameter("rcm_distance_from_tip_m").as_double();
    if (!std::isfinite(rcm_distance) || rcm_distance < 0.02 || rcm_distance > 0.24) {
      RCLCPP_ERROR(get_logger(), "rcm_distance_from_tip_m 必须在 0.02 到 0.24 m 之间");
      return false;
    }

    Eigen::Vector3d rcm_point;
    if (derive_rcm) {
      rcm_point = start_tip - rcm_distance * shaft_axis;
    } else {
      const double x = get_parameter("rcm_x").as_double();
      const double y = get_parameter("rcm_y").as_double();
      const double z = get_parameter("rcm_z").as_double();
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        RCLCPP_ERROR(get_logger(), "固定 RCM 坐标必须是有限数值");
        return false;
      }
      rcm_point = Eigen::Vector3d(x, y, z);
    }

    const double tip_radius = (start_tip - rcm_point).norm();
    const double initial_error = rcmError(start_tip, shaft_axis, rcm_point);
    const double tolerance = get_parameter("rcm_tolerance_m").as_double();
    if (tip_radius > 1.0 || !std::isfinite(tolerance) || tolerance <= 0.0) {
      RCLCPP_ERROR(get_logger(),
        "RCM 参数无效：尖端到 RCM 距离=%.4f m，容差=%.4f m。",
        tip_radius, tolerance);
      return false;
    }
    if (initial_error > tolerance) {
      RCLCPP_WARN(get_logger(),
        "当前姿态暂不满足 RCM：尖端到 RCM 距离=%.4f m，轴线误差=%.4f m，容差=%.4f m。"
        "继续启动，示教回放可先用普通位姿规划移动到记录起始位姿。",
        tip_radius, initial_error, tolerance);
    }
    {
      std::lock_guard<std::mutex> lock(rcm_mutex_);
      rcm_point_ = rcm_point;
    }
    const double eef_step = get_parameter("eef_step_m").as_double();
    if (!std::isfinite(eef_step) || eef_step <= 0.0 || eef_step > 0.01) {
      RCLCPP_ERROR(get_logger(), "参数范围错误：eef_step_m 必须在 (0, 0.01] m 内");
      return false;
    }

    publishPathMarkers(rcm_point, {start_stamped.pose.position});
    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {publishMarkers();});
    publishMotionBusy(false);
    initializeInteractiveTarget(start_stamped.pose);
    state_thread_ = std::thread([this]() {
        while (rclcpp::ok()) {
          publishState();
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
      });
    RCLCPP_INFO(get_logger(),
      "RCM 约束已就绪：倾转单步限幅 %.2f 度，轴向单步限幅 %.3f m；机械臂保持当前位姿",
      get_parameter("max_rcm_rotation_deg").as_double(),
      get_parameter("max_rcm_insertion_m").as_double());
    return true;
  }

private:
  static double rcmError(
    const Eigen::Vector3d & tip,
    const Eigen::Vector3d & axis,
    const Eigen::Vector3d & point)
  {
    const Eigen::Vector3d point_from_tip = point - tip;
    return (point_from_tip - point_from_tip.dot(axis) * axis).norm();
  }

  Eigen::Vector3d currentRcmPoint() const
  {
    std::lock_guard<std::mutex> lock(rcm_mutex_);
    return rcm_point_;
  }

  bool isPoseConsistentWithRcm(
    const geometry_msgs::msg::PoseStamped & pose, double & error) const
  {
    if (pose.header.frame_id != move_group_->getPlanningFrame()) {
      return false;
    }
    const Eigen::Vector3d tip(
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    Eigen::Quaterniond orientation(
      pose.pose.orientation.w, pose.pose.orientation.x,
      pose.pose.orientation.y, pose.pose.orientation.z);
    if (!tip.allFinite() || !std::isfinite(orientation.norm()) ||
      orientation.norm() < 1e-9)
    {
      return false;
    }
    orientation.normalize();
    const Eigen::Vector3d axis = orientation * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d rcm_point = currentRcmPoint();
    error = rcmError(tip, axis, rcm_point);
    const double radius = (tip - rcm_point).norm();
    return std::isfinite(error) && std::isfinite(radius) &&
           error <= get_parameter("rcm_tolerance_m").as_double() &&
           radius <= 1.0;
  }

  bool validateTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const Eigen::Vector3d & rcm_point,
    double & max_error)
  {
    const auto model = move_group_->getRobotModel();
    if (!model) {
      RCLCPP_ERROR(get_logger(), "MoveIt RobotModel 不可用，无法验证 RCM 约束");
      return false;
    }
    moveit::core::RobotState state(model);
    max_error = 0.0;
    const auto & joint_names = trajectory.joint_trajectory.joint_names;
    const double tolerance = get_parameter("rcm_tolerance_m").as_double();

    for (const auto & point : trajectory.joint_trajectory.points) {
      if (point.positions.size() != joint_names.size()) {
        RCLCPP_ERROR(get_logger(), "轨迹关节位置数量异常，无法验证 RCM 约束");
        return false;
      }
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        state.setVariablePosition(joint_names[i], point.positions[i]);
      }
      state.update();
      const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(end_effector_link_);
      const Eigen::Vector3d tip = transform.translation();
      const Eigen::Vector3d axis = transform.linear().col(2).normalized();
      max_error = std::max(max_error, rcmError(tip, axis, rcm_point));
    }

    if (max_error > tolerance) {
      RCLCPP_ERROR(get_logger(),
        "轨迹 RCM 偏差 %.3f mm 超过容差 %.3f mm，拒绝执行",
        max_error * 1000.0, tolerance * 1000.0);
      return false;
    }
    return true;
  }

  void onPerceptionScene(
    const rm_ros_interfaces::msg::PerceptionScene::ConstSharedPtr & scene)
  {
    if (!move_group_ || !planning_scene_interface_) {
      return;
    }
    if (!scene->valid || !scene->transform_valid || !scene->obstacles_valid ||
      scene->header.frame_id != move_group_->getPlanningFrame())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "未同步感知障碍物：scene_valid=%d transform_valid=%d obstacles_valid=%d frame=%s planning_frame=%s",
        scene->valid, scene->transform_valid, scene->obstacles_valid,
        scene->header.frame_id.c_str(), move_group_->getPlanningFrame().c_str());
      return;
    }

    std::map<std::string, std::array<double, 6>> current_bounds;
    std::vector<moveit_msgs::msg::CollisionObject> updates;
    for (const auto & item : scene->objects) {
      if (!item.detected || item.class_name == "liver" || item.class_name == "lesion") {
        continue;
      }
      if (!item.obstacle_geometry_valid) {
        return;
      }
      const auto & lo = item.obstacle_bounds_min_world;
      const auto & hi = item.obstacle_bounds_max_world;
      const std::array<double, 6> bounds{lo.x, lo.y, lo.z, hi.x, hi.y, hi.z};
      if (!std::all_of(bounds.begin(), bounds.end(), [](double value) {
          return std::isfinite(value);
        }) || hi.x <= lo.x || hi.y <= lo.y || hi.z <= lo.z)
      {
        return;
      }

      const std::string id = "perception_" + item.class_name;
      current_bounds[id] = bounds;
      const auto previous = perception_obstacle_bounds_.find(id);
      const double threshold = get_parameter("obstacle_scene_update_threshold_m").as_double();
      if (!std::isfinite(threshold) || threshold <= 0.0) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
          "obstacle_scene_update_threshold_m 必须为正有限值");
        return;
      }
      bool changed = previous == perception_obstacle_bounds_.end();
      if (!changed) {
        for (std::size_t i = 0; i < bounds.size(); ++i) {
          if (std::abs(bounds[i] - previous->second[i]) >= threshold) {
            changed = true;
            break;
          }
        }
      }
      if (!changed) {
        continue;
      }

      moveit_msgs::msg::CollisionObject object;
      object.header = scene->header;
      object.id = id;
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.5 * (lo.x + hi.x);
      pose.position.y = 0.5 * (lo.y + hi.y);
      pose.position.z = 0.5 * (lo.z + hi.z);
      pose.orientation.w = 1.0;
      object.primitives.push_back(box);
      object.primitive_poses.push_back(pose);
      updates.push_back(std::move(object));
    }

    for (const auto & previous : perception_obstacle_bounds_) {
      if (current_bounds.count(previous.first) != 0) {
        continue;
      }
      moveit_msgs::msg::CollisionObject object;
      object.header = scene->header;
      object.id = previous.first;
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      updates.push_back(std::move(object));
    }

    if (!updates.empty() && !planning_scene_interface_->applyCollisionObjects(updates)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
        "MoveIt 拒绝更新感知障碍物场景");
      return;
    }
    perception_obstacle_bounds_ = std::move(current_bounds);
    std_msgs::msg::UInt64 synced_revision;
    synced_revision.data = scene->obstacle_geometry_revision;
    collision_scene_synced_pub_->publish(synced_revision);
  }

  void initializeInteractiveTarget(const geometry_msgs::msg::Pose & pose)
  {
    interactive_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "rcm_demo/interactive", shared_from_this());

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = move_group_->getPlanningFrame();
    marker.pose = pose;
    marker.name = "rcm_constrained_tip_goal";
    marker.description = "RCM 目标：绿色箭头沿器具轴线插入/退出；拖动绿色球可倾转；拖动工具周围的旋转环可绕第六轴滚转";
    marker.scale = 0.12;

    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.name = "tip_visual";
    visual_control.orientation.w = 1.0;
    visual_control.orientation_mode =
      visualization_msgs::msg::InteractiveMarkerControl::VIEW_FACING;
    visual_control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    visual_control.always_visible = true;
    visualization_msgs::msg::Marker tip_marker;
    tip_marker.type = visualization_msgs::msg::Marker::SPHERE;
    tip_marker.pose.orientation.w = 1.0;
    tip_marker.action = visualization_msgs::msg::Marker::ADD;
    tip_marker.scale.x = 0.025;
    tip_marker.scale.y = 0.025;
    tip_marker.scale.z = 0.025;
    tip_marker.color.r = 0.2;
    tip_marker.color.g = 1.0;
    tip_marker.color.b = 0.25;
    tip_marker.color.a = 0.9;
    visual_control.markers.push_back(tip_marker);
    marker.controls.push_back(visual_control);

    addMoveAxisControl(
      marker, "move_tool_axis",
      Eigen::Quaterniond(Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitY())),
      0.1, 0.72, 0.18, true);
    addToolRollControl(marker);

    interactive_server_->insert(
      marker,
      [this](const auto & feedback) {onInteractiveFeedback(feedback);});
    interactive_server_->applyChanges();
    RCLCPP_INFO(get_logger(),
      "RCM 交互目标已就绪：拖动尖端目标并松开，节点会沿器具轴线穿过远心点生成路径并执行");
  }

  static void addMoveAxisControl(
    visualization_msgs::msg::InteractiveMarker & marker,
    const std::string & name,
    const Eigen::Quaterniond & orientation,
    double red,
    double green,
    double blue,
    bool inherit_marker_orientation = false)
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = name;
    control.orientation.x = orientation.x();
    control.orientation.y = orientation.y();
    control.orientation.z = orientation.z();
    control.orientation.w = orientation.w();
    control.orientation_mode = inherit_marker_orientation ?
      visualization_msgs::msg::InteractiveMarkerControl::INHERIT :
      visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    control.always_visible = true;
    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.075;
    arrow.scale.y = 0.012;
    arrow.scale.z = 0.019;
    arrow.color.r = static_cast<float>(red);
    arrow.color.g = static_cast<float>(green);
    arrow.color.b = static_cast<float>(blue);
    arrow.color.a = 1.0;
    control.markers.push_back(arrow);
    marker.controls.push_back(control);
  }

  static void addToolRollControl(visualization_msgs::msg::InteractiveMarker & marker)
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "roll_tool_axis";
    // ROTATE_AXIS uses the control frame's local X axis. Align it with tool +Z.
    const Eigen::Quaterniond control_orientation(
      Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitY()));
    control.orientation.x = control_orientation.x();
    control.orientation.y = control_orientation.y();
    control.orientation.z = control_orientation.z();
    control.orientation.w = control_orientation.w();
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    control.always_visible = true;

    visualization_msgs::msg::Marker ring;
    ring.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ring.action = visualization_msgs::msg::Marker::ADD;
    ring.scale.x = 0.004;
    ring.color.r = 0.25;
    ring.color.g = 0.82;
    ring.color.b = 1.0;
    ring.color.a = 0.95;
    constexpr int segments = 64;
    constexpr double radius = 0.085;
    for (int i = 0; i <= segments; ++i) {
      const double angle = 2.0 * kPi * static_cast<double>(i) / segments;
      geometry_msgs::msg::Point point;
      point.y = radius * std::cos(angle);
      point.z = radius * std::sin(angle);
      ring.points.push_back(point);
    }
    control.markers.push_back(ring);
    marker.controls.push_back(control);
  }

  void onInteractiveFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN) {
      RCLCPP_INFO(get_logger(), "RViz 选中 RCM 控件：%s", feedback->control_name.c_str());
      return;
    }
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      RCLCPP_DEBUG(get_logger(), "RViz 正在拖动 RCM 控件：%s", feedback->control_name.c_str());
      return;
    }
    if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
      return;
    }
    const std::string planning_frame = move_group_->getPlanningFrame();
    RCLCPP_INFO(get_logger(),
      "收到 RCM 交互目标释放事件（control=%s, feedback_frame=%s, planning_frame=%s），开始生成约束路径",
      feedback->control_name.c_str(), feedback->header.frame_id.c_str(), planning_frame.c_str());

    const Eigen::Vector3d dragged_tip(
      feedback->pose.position.x,
      feedback->pose.position.y,
      feedback->pose.position.z);
    const Eigen::Vector3d from_rcm = dragged_tip - currentRcmPoint();
    if (from_rcm.norm() < 1e-6) {
      RCLCPP_WARN(get_logger(), "拖动目标与远心点重合，无法确定器具方向");
      return;
    }

    const Eigen::Vector3d constrained_tip = dragged_tip;
    geometry_msgs::msg::Pose goal;
    goal.position.x = constrained_tip.x();
    goal.position.y = constrained_tip.y();
    goal.position.z = constrained_tip.z();
    goal.orientation = feedback->pose.orientation;
    if (interactive_server_) {
      auto marker_header = feedback->header;
      marker_header.frame_id = planning_frame;
      interactive_server_->setPose(feedback->marker_name, goal, marker_header);
      interactive_server_->applyChanges();
    }
    const Eigen::Quaterniond constrained_orientation(
      goal.orientation.w, goal.orientation.x,
      goal.orientation.y, goal.orientation.z);
    const bool preserve_goal_roll =
      feedback->control_name == "roll_tool_axis" || feedback->control_name == "web_teleop";

    bool expected = false;
    if (!motion_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "上一条 RCM 交互运动仍在执行，请稍候再拖动");
      return;
    }
    publishMotionBusy(true);
    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
    motion_thread_ = std::thread([this, constrained_tip, constrained_orientation, preserve_goal_roll]() {
        bool success = false;
        try {
          success = moveToRcmTarget(constrained_tip, constrained_orientation, preserve_goal_roll);
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(get_logger(), "RCM 交互运动异常：%s", exception.what());
        }
        if (!success) {
          resetInteractiveTargetToCurrentPose();
        }
        motion_active_.store(false);
        publishMotionBusy(false);
      });
  }

  void onWebTarget(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & target)
  {
    if (!interactive_server_) {
      RCLCPP_WARN(get_logger(), "RCM 目标尚未初始化，忽略 Web 遥控命令");
      return;
    }
    auto feedback = std::make_shared<visualization_msgs::msg::InteractiveMarkerFeedback>();
    feedback->event_type = visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP;
    feedback->marker_name = "rcm_constrained_tip_goal";
    feedback->control_name = "web_teleop";
    feedback->client_id = "rcm_web_teleop";
    feedback->header = target->header;
    feedback->pose = target->pose;
    onInteractiveFeedback(feedback);
  }

  void onPlaybackStartTarget(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & target)
  {
    if (!move_group_ || motion_active_.exchange(true)) {
      RCLCPP_DEBUG(get_logger(), "忽略重复的示教起始位姿请求：当前已有运动");
      return;
    }
    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
    publishMotionBusy(true);
    const auto requested = *target;
    motion_thread_ = std::thread([this, requested]() {
      bool success = false;
      move_group_->clearPoseTargets();
      move_group_->setPoseTarget(requested, end_effector_link_);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto plan_result = move_group_->plan(plan);
      if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(get_logger(),
          "示教起始位姿精确 IK 规划失败（错误码=%d），尝试近似 IK",
          plan_result.val);
        move_group_->clearPoseTargets();
        if (move_group_->setApproximateJointValueTarget(requested, end_effector_link_)) {
          plan_result = move_group_->plan(plan);
        }
      }
      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
        const auto execute_result = move_group_->execute(plan);
        success = execute_result == moveit::core::MoveItErrorCode::SUCCESS;
        if (!success) {
          RCLCPP_WARN(get_logger(), "示教起始位姿轨迹执行失败（错误码=%d）", execute_result.val);
        }
      } else {
        RCLCPP_WARN(get_logger(), "示教起始位姿近似 IK 规划失败（错误码=%d）", plan_result.val);
      }
      move_group_->clearPoseTargets();
      publishMotionBusy(false);
      motion_active_.store(false);
      publishPlaybackStartResult(requested, success);
      RCLCPP_INFO(get_logger(), "示教起始位姿恢复%s（此步骤不检查当前 RCM 偏差）",
        success ? "完成" : "失败");
    });
  }

  void onPlaybackStartJoints(
    const sensor_msgs::msg::JointState::ConstSharedPtr & target)
  {
    if (!move_group_ || motion_active_.exchange(true)) {
      RCLCPP_DEBUG(get_logger(), "忽略重复的关节起始位姿请求：当前已有运动");
      return;
    }
    if (motion_thread_.joinable()) {
      motion_thread_.join();
    }
    publishMotionBusy(true);
    const auto requested = *target;
    motion_thread_ = std::thread([this, requested]() {
      bool success = false;
      move_group_->clearPoseTargets();
      std::map<std::string, double> joint_target;
      const auto active_joints = move_group_->getActiveJoints();
      for (std::size_t i = 0; i < requested.name.size() && i < requested.position.size(); ++i) {
        std::string joint_name = requested.name[i];
        if (std::find(active_joints.begin(), active_joints.end(), joint_name) == active_joints.end()) {
          const std::string namespace_name = get_namespace();
          const auto slash = namespace_name.find_last_of('/');
          if (slash != std::string::npos && slash + 1 < namespace_name.size()) {
            std::string arm_prefix = namespace_name.substr(slash + 1);
            const auto arm_suffix = arm_prefix.find("_arm");
            if (arm_suffix != std::string::npos) {
              arm_prefix.erase(arm_suffix);
            }
            arm_prefix += "_";
            if (joint_name.rfind(arm_prefix, 0) == 0) {
              const std::string stripped = joint_name.substr(arm_prefix.size());
              if (std::find(active_joints.begin(), active_joints.end(), stripped) != active_joints.end()) {
                joint_name = stripped;
              }
            }
          }
        }
        joint_target[joint_name] = requested.position[i];
      }
      if (joint_target.size() == active_joints.size() &&
        std::all_of(active_joints.begin(), active_joints.end(),
          [&joint_target](const auto & name) {return joint_target.count(name) != 0;})) {
        move_group_->setJointValueTarget(joint_target);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const auto plan_result = move_group_->plan(plan);
        if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
          const auto execute_result = move_group_->execute(plan);
          success = execute_result == moveit::core::MoveItErrorCode::SUCCESS;
          if (!success) {
            RCLCPP_WARN(get_logger(), "示教关节起始位姿执行失败（错误码=%d）", execute_result.val);
          }
        } else {
          RCLCPP_WARN(get_logger(), "示教关节起始位姿规划失败（错误码=%d）", plan_result.val);
        }
      } else {
        RCLCPP_WARN(get_logger(), "示教关节起始位姿关节数量不匹配：收到 %zu，期望 %zu",
          joint_target.size(), active_joints.size());
      }
      publishMotionBusy(false);
      motion_active_.store(false);
      publishPlaybackStartResult(requested.header.stamp, success);
      RCLCPP_INFO(get_logger(), "示教关节起始位姿恢复%s",
        success ? "完成" : "失败");
    });
  }

  void publishPlaybackStartResult(
    const geometry_msgs::msg::PoseStamped & pose, bool success)
  {
    publishPlaybackStartResult(pose.header.stamp, success);
  }

  void publishPlaybackStartResult(
    const builtin_interfaces::msg::Time & stamp, bool success)
  {
    std_msgs::msg::UInt64 result;
    result.data = (static_cast<uint64_t>(stamp.sec) << 32) |
      static_cast<uint32_t>(stamp.nanosec);
    if (!success) {
      result.data |= (uint64_t{1} << 63);
    }
    playback_start_result_pub_->publish(result);
  }

  bool moveToRcmTarget(
    const Eigen::Vector3d & target_tip, Eigen::Quaterniond goal_orientation,
    bool preserve_goal_roll)
  {
    const Eigen::Vector3d rcm_point = currentRcmPoint();
    geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose(end_effector_link_);
    double current_rcm_error = 0.0;
    if (!isPoseConsistentWithRcm(current, current_rcm_error)) {
      RCLCPP_ERROR(get_logger(),
        "MoveIt 当前尖端位姿无效或不满足 RCM（frame=%s，位置=[%.3f, %.3f, %.3f] m，"
        "RCM 轴线偏差=%.3f mm）；停止规划并检查 joint_states 时间戳/MoveIt 状态监视",
        current.header.frame_id.c_str(), current.pose.position.x,
        current.pose.position.y, current.pose.position.z, current_rcm_error * 1000.0);
      return false;
    }
    Eigen::Vector3d start_tip(
      current.pose.position.x, current.pose.position.y, current.pose.position.z);
    Eigen::Quaterniond start_orientation(
      current.pose.orientation.w,
      current.pose.orientation.x,
      current.pose.orientation.y,
      current.pose.orientation.z);
    if (start_orientation.norm() < 1e-9) {
      RCLCPP_ERROR(get_logger(), "无法读取当前器具姿态，取消 RCM 交互运动");
      return false;
    }
    start_orientation.normalize();
    if (!std::isfinite(goal_orientation.norm()) || goal_orientation.norm() < 1e-9) {
      RCLCPP_ERROR(get_logger(), "RCM 目标末端姿态无效");
      return false;
    }
    goal_orientation.normalize();
    const Eigen::Vector3d start_axis = start_orientation * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d start_from_rcm = start_tip - rcm_point;
    const double start_radius = start_from_rcm.norm();
    const Eigen::Vector3d goal_from_rcm = target_tip - rcm_point;
    double goal_radius = goal_from_rcm.norm();
    const bool initializing = start_radius < 0.02;
    if (!std::isfinite(goal_radius) || goal_radius < 1e-4 || goal_radius > 1.0) {
      RCLCPP_WARN(get_logger(),
        "目标尖端到 RCM 的距离 %.3f m 超出允许范围 (0, 1.0] m，不执行", goal_radius);
      return false;
    }
    Eigen::Vector3d goal_axis = goal_from_rcm / goal_radius;
    double axis_angle = std::acos(std::clamp(start_axis.dot(goal_axis), -1.0, 1.0));
    if (initializing) {
      const double orientation_change = start_orientation.angularDistance(goal_orientation);
      if (axis_angle > 0.1 * kPi / 180.0 ||
        orientation_change > 0.1 * kPi / 180.0)
      {
        RCLCPP_WARN(get_logger(),
          "尖端距 RCM 不足 20 mm，初始化阶段只允许沿器具轴线进退；倾转和第六轴滚转已锁定");
        return false;
      }
      goal_axis = start_axis;
    }
    // Treat only near-zero angular changes as insertion/retraction. The web
    // joystick sends 2-degree tilt steps, so a wider threshold misclassifies
    // every joystick command as axial motion and suppresses the tilt.
    const bool axial_motion = initializing || axis_angle < 0.5 * kPi / 180.0;
    if (axial_motion) {
      goal_radius = goal_from_rcm.dot(start_axis);
      const double min_radius = initializing ? 1e-4 : 0.02;
      if (goal_radius < min_radius || goal_radius > 1.0) {
        RCLCPP_WARN(get_logger(), "轴向目标投影距离 %.3f m 超出允许范围，不执行", goal_radius);
        return false;
      }
      goal_axis = start_axis;
      axis_angle = 0.0;
    }
    const double max_rotation = get_parameter("max_rcm_rotation_deg").as_double();
    const double max_insertion = get_parameter("max_rcm_insertion_m").as_double();
    if (!std::isfinite(max_rotation) || max_rotation < 0.0 ||
      !std::isfinite(max_insertion) || max_insertion < 0.0)
    {
      RCLCPP_ERROR(get_logger(), "RCM 单次运动限幅参数必须是非负有限值");
      return false;
    }
    double insertion_delta = std::abs(goal_radius - start_radius);
    constexpr double insertion_limit_tolerance_m = 1e-6;
    if (max_rotation > 0.0 && axis_angle > max_rotation * kPi / 180.0) {
      RCLCPP_WARN(get_logger(),
        "RCM 倾转目标 %.2f 度超过单次限幅 %.2f 度，不执行；请在 RViz 中减小拖动量",
        axis_angle * 180.0 / kPi, max_rotation);
      return false;
    }
    if (max_insertion > 0.0 &&
      insertion_delta > max_insertion + insertion_limit_tolerance_m)
    {
      const double direction = goal_radius >= start_radius ? 1.0 : -1.0;
      const double requested_delta = insertion_delta;
      goal_radius = start_radius + direction * max_insertion;
      insertion_delta = max_insertion;
      RCLCPP_INFO(get_logger(),
        "RCM 轴向目标变化 %.3f mm 超过单次限幅 %.3f mm，已自动裁剪到限幅内",
        requested_delta * 1000.0, max_insertion * 1000.0);
    }
    RCLCPP_INFO(get_logger(),
      "RCM 轴向行程：当前尖端距远心点 %.3f m，目标 %.3f m，变化 %.3f m；轴向夹角 %.1f 度",
      start_radius, goal_radius, goal_radius - start_radius, axis_angle * 180.0 / kPi);
    const double eef_step = get_parameter("eef_step_m").as_double();
    double max_error = 0.0;
    moveit_msgs::msg::RobotTrajectory trajectory;
    std::vector<geometry_msgs::msg::Point> tip_path;
    std::vector<Eigen::Vector3d> route_midpoints;
    route_midpoints.push_back(goal_axis);

    Eigen::Vector3d side_axis = start_axis.cross(goal_axis);
    if (side_axis.norm() < 1e-6) {
      const Eigen::Vector3d reference =
        std::abs(start_axis.dot(Eigen::Vector3d::UnitZ())) < 0.9 ?
        Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
      side_axis = start_axis.cross(reference);
    }
    side_axis.normalize();
    route_midpoints.push_back(side_axis);
    route_midpoints.push_back(-side_axis);

    bool path_found = false;
    bool partial_axial_path = false;
    double selected_fraction = 1.0;
    const std::size_t attempts = axial_motion ? 1 : route_midpoints.size();
    for (std::size_t attempt = 0; attempt < attempts; ++attempt) {
      std::vector<Eigen::Vector3d> route;
      if (attempt == 0) {
        route.push_back(goal_axis);
      } else {
        route.push_back(route_midpoints[attempt]);
        route.push_back(goal_axis);
      }

      Eigen::Quaterniond route_end_orientation = start_orientation;
      Eigen::Vector3d route_start_axis = start_axis;
      for (const Eigen::Vector3d & route_axis : route) {
        Eigen::Quaterniond route_rotation = Eigen::Quaterniond::FromTwoVectors(
          route_start_axis, route_axis);
        route_rotation.normalize();
        route_end_orientation = route_rotation * route_end_orientation;
        route_end_orientation.normalize();
        route_start_axis = route_axis;
      }
      Eigen::Quaterniond roll_residual = preserve_goal_roll ?
        route_end_orientation.inverse() * goal_orientation : Eigen::Quaterniond::Identity();
      roll_residual.normalize();
      if (roll_residual.w() < 0.0) {
        roll_residual.coeffs() *= -1.0;
      }
      const double roll_angle = Eigen::AngleAxisd(roll_residual).angle();

      std::vector<geometry_msgs::msg::Pose> waypoints;
      std::vector<geometry_msgs::msg::Point> candidate_tip_path;
      candidate_tip_path.push_back(current.pose.position);
      Eigen::Vector3d segment_start_axis = start_axis;
      Eigen::Quaterniond segment_start_orientation = start_orientation;
      double total_angle = 0.0;
      for (const Eigen::Vector3d & segment_goal_axis : route) {
        const Eigen::Quaterniond segment_rotation = Eigen::Quaterniond::FromTwoVectors(
          segment_start_axis, segment_goal_axis);
        const double segment_angle = Eigen::AngleAxisd(segment_rotation.normalized()).angle();
        total_angle += segment_angle;
        segment_start_axis = segment_goal_axis;
      }
      segment_start_axis = start_axis;
      double completed_angle = 0.0;
      double completed_fraction = 0.0;
      for (const Eigen::Vector3d & segment_goal_axis : route) {
        Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
          segment_start_axis, segment_goal_axis);
        rotation.normalize();
        const double angle = Eigen::AngleAxisd(rotation).angle();
        const double segment_end_fraction = total_angle > 1e-9 ?
          (completed_angle + angle) / total_angle : 1.0;
        const double segment_start_fraction = completed_fraction;
        const double segment_radius_start = start_radius +
          (goal_radius - start_radius) * segment_start_fraction;
        const double segment_radius_end = start_radius +
          (goal_radius - start_radius) * segment_end_fraction;
        const double segment_roll_angle = roll_angle *
          (segment_end_fraction - segment_start_fraction);
        const int steps = std::max(
          2, static_cast<int>(std::ceil(std::max(
          std::max((angle + segment_roll_angle) / (3.0 * kPi / 180.0),
          std::max(segment_radius_start, segment_radius_end) * angle / eef_step),
          std::abs(segment_radius_end - segment_radius_start) / eef_step))));
        for (int i = 1; i <= steps; ++i) {
          const double fraction = static_cast<double>(i) / steps;
          Eigen::Quaterniond orientation =
            Eigen::Quaterniond::Identity().slerp(fraction, rotation) *
            segment_start_orientation;
          const double roll_fraction = segment_start_fraction +
            (segment_end_fraction - segment_start_fraction) * fraction;
          orientation *= Eigen::Quaterniond::Identity().slerp(
            roll_fraction, roll_residual);
          orientation.normalize();
          const Eigen::Vector3d axis = orientation * Eigen::Vector3d::UnitZ();
          const double path_fraction = segment_start_fraction +
            (segment_end_fraction - segment_start_fraction) * fraction;
          const double radius = start_radius + (goal_radius - start_radius) * path_fraction;
          const Eigen::Vector3d tip = rcm_point + radius * axis;

          geometry_msgs::msg::Pose pose;
          pose.position.x = tip.x();
          pose.position.y = tip.y();
          pose.position.z = tip.z();
          pose.orientation.x = orientation.x();
          pose.orientation.y = orientation.y();
          pose.orientation.z = orientation.z();
          pose.orientation.w = orientation.w();
          waypoints.push_back(pose);
          candidate_tip_path.push_back(pose.position);
        }
        segment_start_orientation = rotation * segment_start_orientation;
        segment_start_orientation.normalize();
        segment_start_axis = segment_goal_axis;
        completed_angle += angle;
        completed_fraction = segment_end_fraction;
      }

      move_group_->setStartStateToCurrentState();
      moveit_msgs::msg::RobotTrajectory candidate_trajectory;
      moveit_msgs::msg::MoveItErrorCodes cartesian_error;
      const double fraction = move_group_->computeCartesianPath(
        waypoints, eef_step, 0.0, candidate_trajectory, true, &cartesian_error);
      RCLCPP_INFO(get_logger(),
        "RCM 路径方案 %zu 完成度 %.1f%%，MoveIt 错误码=%d",
        attempt + 1, fraction * 100.0, cartesian_error.val);
      if (candidate_trajectory.joint_trajectory.points.empty()) {
        continue;
      }
      const bool is_complete = fraction >= 0.999;
      const bool acceptable_axial_prefix = axial_motion && fraction >= 0.85;
      if (!is_complete && !acceptable_axial_prefix) {
        continue;
      }

      double candidate_max_error = 0.0;
      if (!validateTrajectory(candidate_trajectory, rcm_point, candidate_max_error)) {
        continue;
      }
      trajectory = std::move(candidate_trajectory);
      tip_path = std::move(candidate_tip_path);
      if (!is_complete) {
        partial_axial_path = true;
        selected_fraction = fraction;
        const std::size_t last_path_index = std::min(
          tip_path.size() - 1,
          static_cast<std::size_t>(std::floor(fraction * (tip_path.size() - 1))));
        tip_path.resize(std::max<std::size_t>(2, last_path_index + 1));
      }
      max_error = candidate_max_error;
      path_found = true;
      break;
    }
    if (!path_found) {
      if (axial_motion) {
        RCLCPP_WARN(get_logger(),
          "轴向路径未完整规划（当前 %.3f m，目标 %.3f m）；可能超出机械臂可达行程或碰撞约束，不执行",
          start_radius, goal_radius);
      } else {
        RCLCPP_WARN(get_logger(),
          "直达路径及两条 RCM 约束绕行路径均未完整规划；目标可能超出工作空间或被碰撞约束阻挡，不执行");
      }
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    const auto result = move_group_->execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "RCM 交互轨迹执行失败，MoveIt 错误码=%d", result.val);
      return false;
    }

    publishPathMarkers(rcm_point, tip_path);
    if (partial_axial_path) {
      RCLCPP_WARN(get_logger(),
        "轴向路径的可达部分已执行（%.1f%%），机械臂停在当前可达位置；最大 RCM 偏差 %.3f mm",
        selected_fraction * 100.0, max_error * 1000.0);
    } else {
      RCLCPP_INFO(get_logger(), "RCM 交互目标运动完成，最大采样偏差 %.3f mm",
        max_error * 1000.0);
    }
    return true;
  }

  void resetInteractiveTargetToCurrentPose()
  {
    if (!interactive_server_) {
      return;
    }
    const auto current = move_group_->getCurrentPose(end_effector_link_);
    double current_rcm_error = 0.0;
    if (!isPoseConsistentWithRcm(current, current_rcm_error)) {
      RCLCPP_WARN(get_logger(),
        "当前尖端状态不可用，保留 RCM 交互目标；RCM 轴线偏差 %.3f mm",
        current_rcm_error * 1000.0);
      return;
    }
    auto marker_header = current.header;
    marker_header.frame_id = move_group_->getPlanningFrame();
    interactive_server_->setPose("rcm_constrained_tip_goal", current.pose, marker_header);
    interactive_server_->applyChanges();
  }

  void publishPathMarkers(
    const Eigen::Vector3d & rcm_point,
    const std::vector<geometry_msgs::msg::Point> & tip_path)
  {
    visualization_msgs::msg::MarkerArray updated_markers;
    const std::string frame = move_group_->getPlanningFrame();

    visualization_msgs::msg::Marker pivot;
    pivot.header.frame_id = frame;
    pivot.ns = "rcm";
    pivot.id = 0;
    pivot.type = visualization_msgs::msg::Marker::SPHERE;
    pivot.action = visualization_msgs::msg::Marker::ADD;
    pivot.pose.position.x = rcm_point.x();
    pivot.pose.position.y = rcm_point.y();
    pivot.pose.position.z = rcm_point.z();
    pivot.pose.orientation.w = 1.0;
    pivot.scale.x = 0.018;
    pivot.scale.y = 0.018;
    pivot.scale.z = 0.018;
    pivot.color.r = 1.0;
    pivot.color.g = 0.15;
    pivot.color.b = 0.1;
    pivot.color.a = 1.0;
    updated_markers.markers.push_back(pivot);

    visualization_msgs::msg::Marker path;
    path.header.frame_id = frame;
    path.ns = "rcm";
    path.id = 1;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.scale.x = 0.003;
    path.color.r = 0.1;
    path.color.g = 0.9;
    path.color.b = 0.2;
    path.color.a = 1.0;
    path.points = tip_path;
    updated_markers.markers.push_back(path);

    visualization_msgs::msg::Marker shafts;
    shafts.header.frame_id = frame;
    shafts.ns = "rcm";
    shafts.id = 2;
    shafts.type = visualization_msgs::msg::Marker::LINE_LIST;
    shafts.action = visualization_msgs::msg::Marker::ADD;
    shafts.pose.orientation.w = 1.0;
    shafts.scale.x = 0.0015;
    shafts.color.r = 0.1;
    shafts.color.g = 0.5;
    shafts.color.b = 1.0;
    shafts.color.a = 0.55;
    geometry_msgs::msg::Point pivot_point;
    pivot_point.x = rcm_point.x();
    pivot_point.y = rcm_point.y();
    pivot_point.z = rcm_point.z();
    for (const auto & tip : tip_path) {
      shafts.points.push_back(pivot_point);
      shafts.points.push_back(tip);
    }
    updated_markers.markers.push_back(shafts);
    {
      std::lock_guard<std::mutex> lock(markers_mutex_);
      markers_ = std::move(updated_markers);
    }
    publishMarkers();
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray marker_snapshot;
    {
      std::lock_guard<std::mutex> lock(markers_mutex_);
      marker_snapshot = markers_;
    }
    if (marker_snapshot.markers.empty()) {
      return;
    }
    const auto stamp = now();
    for (auto & marker : marker_snapshot.markers) {
      marker.header.stamp = stamp;
    }
    marker_pub_->publish(marker_snapshot);
  }

  void publishState()
  {
    if (!move_group_) {
      return;
    }
    auto pose = move_group_->getCurrentPose(end_effector_link_);
    current_pose_pub_->publish(pose);
    geometry_msgs::msg::PointStamped rcm;
    rcm.header.frame_id = move_group_->getPlanningFrame();
    rcm.header.stamp = now();
    const Eigen::Vector3d rcm_point = currentRcmPoint();
    rcm.point.x = rcm_point.x();
    rcm.point.y = rcm_point.y();
    rcm.point.z = rcm_point.z();
    rcm_point_pub_->publish(rcm);
  }

  void publishMotionBusy(bool busy)
  {
    std_msgs::msg::Bool message;
    message.data = busy;
    motion_busy_pub_->publish(message);
  }

  void onSetRcmPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr & message)
  {
    std_msgs::msg::Bool result;
    result.data = false;
    if (motion_active_.load()) {
      RCLCPP_WARN(get_logger(), "拒绝更新 RCM：当前仍有约束运动执行");
    } else if (message->header.frame_id != move_group_->getPlanningFrame()) {
      RCLCPP_WARN(get_logger(), "拒绝更新 RCM：坐标系 %s 与规划坐标系 %s 不一致",
        message->header.frame_id.c_str(), move_group_->getPlanningFrame().c_str());
    } else {
      const Eigen::Vector3d point(message->point.x, message->point.y, message->point.z);
      const auto current_state = move_group_->getCurrentState(2.0);
      if (!current_state) {
        RCLCPP_WARN(get_logger(), "拒绝更新 RCM：等待最新关节状态超时，未使用过期位姿");
      } else if (point.allFinite()) {
        const Eigen::Isometry3d & tip_transform =
          current_state->getGlobalLinkTransform(end_effector_link_);
        const Eigen::Vector3d tip = tip_transform.translation();
        Eigen::Quaterniond orientation(tip_transform.linear());
        orientation.normalize();
        const Eigen::Vector3d axis = orientation * Eigen::Vector3d::UnitZ();
        const double error = rcmError(tip, axis, point);
        const double radius = (tip - point).norm();
        const double tolerance = std::min(0.005, get_parameter("rcm_tolerance_m").as_double());
        if (std::isfinite(error) && radius <= 1.0 && error <= tolerance) {
          {
            std::lock_guard<std::mutex> lock(rcm_mutex_);
            rcm_point_ = point;
          }
          set_parameter(rclcpp::Parameter("derive_rcm_from_current", false));
          set_parameter(rclcpp::Parameter("rcm_x", point.x()));
          set_parameter(rclcpp::Parameter("rcm_y", point.y()));
          set_parameter(rclcpp::Parameter("rcm_z", point.z()));
          geometry_msgs::msg::Pose current_pose;
          current_pose.position.x = tip.x();
          current_pose.position.y = tip.y();
          current_pose.position.z = tip.z();
          current_pose.orientation.x = orientation.x();
          current_pose.orientation.y = orientation.y();
          current_pose.orientation.z = orientation.z();
          current_pose.orientation.w = orientation.w();
          publishPathMarkers(point, {current_pose.position});
          std_msgs::msg::Header marker_header;
          marker_header.frame_id = move_group_->getPlanningFrame();
          marker_header.stamp = now();
          interactive_server_->setPose("rcm_constrained_tip_goal", current_pose, marker_header);
          interactive_server_->applyChanges();
          result.data = true;
          RCLCPP_INFO(get_logger(),
            "RCM 标定点已应用：[%.4f, %.4f, %.4f] m，尖端距离 %.3f m，轴线偏差 %.2f mm",
            point.x(), point.y(), point.z(), radius, error * 1000.0);
        } else {
          RCLCPP_WARN(get_logger(),
            "拒绝更新 RCM：当前尖端距离 %.3f m（需不超过 1.0 m），轴线偏差 %.2f mm（限值 %.2f mm）",
            radius, error * 1000.0, tolerance * 1000.0);
        }
      }
    }
    set_rcm_result_pub_->publish(result);
  }

  std::string end_effector_link_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rcm_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_busy_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_rcm_result_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr playback_start_result_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr collision_scene_synced_pub_;
  rclcpp::CallbackGroup::SharedPtr set_rcm_point_callback_group_;
  rclcpp::CallbackGroup::SharedPtr perception_scene_callback_group_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr web_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr playback_start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr playback_start_joints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr set_rcm_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Subscription<rm_ros_interfaces::msg::PerceptionScene>::SharedPtr perception_scene_sub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
  visualization_msgs::msg::MarkerArray markers_;
  std::mutex markers_mutex_;
  std::map<std::string, std::array<double, 6>> perception_obstacle_bounds_;
  mutable std::mutex rcm_mutex_;
  std::atomic<bool> motion_active_{false};
  std::thread motion_thread_;
  std::thread state_thread_;
  Eigen::Vector3d rcm_point_{Eigen::Vector3d::Zero()};
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<Rm65RcmDemo>(options);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  bool success = node->init() && node->runDemo();
  if (!success) {
    rclcpp::shutdown();
    spinner.join();
    node->waitForStatePublisher();
    node->waitForInteractiveMotion();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "RCM demo 保持运行以持续显示远心点；按 Ctrl-C 退出");
  spinner.join();
  node->waitForStatePublisher();
  node->waitForInteractiveMotion();
  return 0;
}
