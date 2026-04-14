#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "mrr_pcd_create_msgs/msg/robot_announcement.hpp"
#include "mrr_pcd_create_msgs/action/capture_pcd.hpp"

enum class State { IDLE, RECORDING, DONE, ERR };

struct ImuSample {
  double x, y, z;
};

struct PointCloudCapture {
  sensor_msgs::msg::PointCloud2 accumulated;
  bool accumulated_valid = false;
  size_t cloud_count = 0;
  bool done = false;
};

using CapturePcd = mrr_pcd_create_msgs::action::CapturePcd;
using GoalHandleCapturePcd = rclcpp_action::ServerGoalHandle<CapturePcd>;

// ---------------------------------------------------------------------------

class SlaveNode : public rclcpp::Node {
public:
  SlaveNode() : Node("slave_node") {
    // Use robot namespace as robot identity.
    this->declare_parameter<std::string>("robot_ns", "");
    std::string configured_robot_ns = this->get_parameter("robot_ns").as_string();
    if (configured_robot_ns.empty()) {
      configured_robot_ns = this->get_namespace();
    }
    robot_ns_ = normalize_namespace(configured_robot_ns);

    // Publisher for robot announcements
    announcement_pub_ = this->create_publisher<mrr_pcd_create_msgs::msg::RobotAnnouncement>(
      "/mrr_pcd_announcement", rclcpp::SystemDefaultsQoS());

    capture_action_name_ = (robot_ns_ == "/") ? "/mrr_pcd_capture" : (robot_ns_ + "/mrr_pcd_capture");

    // Action server
    action_server_ = rclcpp_action::create_server<CapturePcd>(
      this,
      capture_action_name_,
      std::bind(&SlaveNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SlaveNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&SlaveNode::handle_accepted, this, std::placeholders::_1));

    // Timer for periodic announcements (every 2 seconds)
    announce_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SlaveNode::publish_announcement, this));

    RCLCPP_INFO(this->get_logger(), "Slave node for namespace '%s' started (capture action: %s)",
                robot_ns_.c_str(), capture_action_name_.c_str());
  }

private:
  std::string robot_ns_;
  std::string capture_action_name_;
  rclcpp::Publisher<mrr_pcd_create_msgs::msg::RobotAnnouncement>::SharedPtr announcement_pub_;
  rclcpp_action::Server<CapturePcd>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr announce_timer_;

  static std::string normalize_namespace(std::string ns) {
    if (ns.empty()) {
      return "/";
    }
    if (ns.front() != '/') {
      ns.insert(ns.begin(), '/');
    }
    while (ns.size() > 1 && ns.back() == '/') {
      ns.pop_back();
    }
    return ns;
  }

  bool topic_in_robot_namespace(const std::string & topic) const {
    if (robot_ns_ == "/") {
      return !topic.empty() && topic.front() == '/';
    }
    if (topic.rfind(robot_ns_, 0) != 0) {
      return false;
    }
    if (topic.size() == robot_ns_.size()) {
      return true;
    }
    return topic[robot_ns_.size()] == '/';
  }

  bool node_namespace_in_robot_namespace(const std::string & node_ns) const {
    const std::string normalized = normalize_namespace(node_ns);
    if (robot_ns_ == "/") {
      return !normalized.empty() && normalized.front() == '/';
    }
    if (normalized.rfind(robot_ns_, 0) != 0) {
      return false;
    }
    if (normalized.size() == robot_ns_.size()) {
      return true;
    }
    return normalized[robot_ns_.size()] == '/';
  }

  static bool has_xyz_fields(const sensor_msgs::msg::PointCloud2 & cloud) {
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto & field : cloud.fields) {
      has_x = has_x || field.name == "x";
      has_y = has_y || field.name == "y";
      has_z = has_z || field.name == "z";
    }
    return has_x && has_y && has_z;
  }

  static bool parse_transform_matrix(
    const std::string & text,
    Eigen::Matrix4f & lidar_1_to_2,
    std::string & error_msg)
  {
    std::string sanitized = text;
    for (char & ch : sanitized) {
      if (ch == ',' || ch == ';' || ch == '[' || ch == ']') {
        ch = ' ';
      }
    }

    std::stringstream ss(sanitized);
    std::array<double, 16> values{};
    size_t count = 0;
    while (ss && count < values.size()) {
      ss >> values[count];
      if (ss.fail()) {
        ss.clear();
        char ignored = '\0';
        ss >> ignored;
        if (!ss) {
          break;
        }
        if (!std::isspace(static_cast<unsigned char>(ignored))) {
          error_msg = "Transform matrix contains invalid characters";
          return false;
        }
        continue;
      }
      ++count;
    }

    double extra_value = 0.0;
    if ((ss >> extra_value) || count != values.size()) {
      error_msg = "Transform matrix must contain exactly 16 numeric values";
      return false;
    }

    lidar_1_to_2 = Eigen::Matrix4f::Identity();
    for (size_t row = 0; row < 4; ++row) {
      for (size_t col = 0; col < 4; ++col) {
        lidar_1_to_2(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          static_cast<float>(values[row * 4 + col]);
      }
    }
    return true;
  }

  static sensor_msgs::msg::PointCloud2 transform_cloud_xyz(
    const sensor_msgs::msg::PointCloud2 & input,
    const Eigen::Matrix4f & transform)
  {
    sensor_msgs::msg::PointCloud2 output = input;
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const Eigen::Vector4f point(*iter_x, *iter_y, *iter_z, 1.0f);
      const Eigen::Vector4f transformed = transform * point;
      *iter_x = transformed.x();
      *iter_y = transformed.y();
      *iter_z = transformed.z();
    }

    return output;
  }

  static bool save_cloud_to_pcd(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const std::string & pcd_path,
    size_t & num_points)
  {
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud, pcl_cloud);
    pcl::PCDWriter writer;
    if (writer.writeBinary(pcd_path, pcl_cloud) != 0) {
      return false;
    }

    num_points = static_cast<size_t>(pcl_cloud.width) * static_cast<size_t>(pcl_cloud.height);
    return true;
  }

  static void append_cloud_sample(
    PointCloudCapture & capture,
    const sensor_msgs::msg::PointCloud2 & msg,
    double elapsed,
    float acc_time_s)
  {
    if (capture.done) {
      return;
    }

    if (acc_time_s <= 0.0f) {
      capture.accumulated = msg;
      capture.accumulated_valid = true;
      capture.cloud_count = 1;
      capture.done = true;
      return;
    }

    if (!capture.accumulated_valid) {
      capture.accumulated = msg;
      capture.accumulated_valid = true;
    } else {
      sensor_msgs::msg::PointCloud2 merged;
      pcl::concatenatePointCloud(capture.accumulated, msg, merged);
      capture.accumulated = std::move(merged);
    }
    ++capture.cloud_count;

    if (elapsed >= static_cast<double>(acc_time_s)) {
      capture.done = true;
    }
  }

  // ---------- Announcement ----------
  void publish_announcement() {
    auto ann = mrr_pcd_create_msgs::msg::RobotAnnouncement();
    ann.robot_ns = robot_ns_;
    ann.capture_action_name = capture_action_name_;

    // Discover available topics visible to this slave.
    // Do not filter by namespace here: topic namespaces can differ from the
    // robot identity namespace and would otherwise hide valid lidar/IMU topics.
    for (const auto& [topic, types] : this->get_topic_names_and_types()) {
      for (const auto& t : types) {
        if (t == "sensor_msgs/msg/PointCloud2") {
          if (std::find(ann.available_pc_topics.begin(), ann.available_pc_topics.end(), topic) ==
              ann.available_pc_topics.end()) {
            ann.available_pc_topics.push_back(topic);
          }
        }
        if (t == "sensor_msgs/msg/Imu") {
          if (std::find(ann.available_imu_topics.begin(), ann.available_imu_topics.end(), topic) ==
              ann.available_imu_topics.end()) {
            ann.available_imu_topics.push_back(topic);
          }
        }
      }
    }
    std::sort(ann.available_pc_topics.begin(), ann.available_pc_topics.end());
    std::sort(ann.available_imu_topics.begin(), ann.available_imu_topics.end());

    announcement_pub_->publish(ann);
  }

  // ---------- Action Server ----------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const CapturePcd::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "Received capture goal: PC1=%s, PC2=%s, IMU=%s, acc_time=%.1f, dual=%s",
                goal->pc_topic.c_str(), goal->pc_topic_2.c_str(), goal->imu_topic.c_str(),
                goal->acc_time_s, goal->use_second_lidar ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCapturePcd> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCapturePcd> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted, starting capture worker thread");
    // Create a new thread to execute the goal
    std::thread{std::bind(&SlaveNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCapturePcd> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CapturePcd::Feedback>();
    auto result = std::make_shared<CapturePcd::Result>();

    result->robot_ns = robot_ns_;
    result->success = false;

    RCLCPP_INFO(this->get_logger(),
                "Starting capture execution: lidar1='%s', lidar2='%s', imu='%s', use_second_lidar=%s, use_imu=%s",
                goal->pc_topic.c_str(), goal->pc_topic_2.c_str(), goal->imu_topic.c_str(),
                goal->use_second_lidar ? "true" : "false",
                goal->use_imu ? "true" : "false");

    if (goal->pc_topic.empty()) {
      result->error_msg = "Lidar 1 pointcloud topic is empty";
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }
    if (goal->use_second_lidar && goal->pc_topic_2.empty()) {
      result->error_msg = "Lidar 2 pointcloud topic is empty";
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }

    Eigen::Matrix4f lidar_2_to_1 = Eigen::Matrix4f::Identity();
    if (goal->use_second_lidar) {
      Eigen::Matrix4f lidar_1_to_2 = Eigen::Matrix4f::Identity();
      std::string transform_error;
      if (!parse_transform_matrix(goal->lidar_1_to_2_transform, lidar_1_to_2, transform_error)) {
        result->error_msg = transform_error;
        RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
        goal_handle->abort(result);
        return;
      }
      if (!lidar_1_to_2.fullPivLu().isInvertible()) {
        result->error_msg = "Transform matrix is not invertible";
        RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
        goal_handle->abort(result);
        return;
      }
      lidar_2_to_1 = lidar_1_to_2.inverse();
    }

    // Setup recording
    State state = State::RECORDING;
    (void)state;
    PointCloudCapture lidar_1_capture;
    PointCloudCapture lidar_2_capture;
    std::vector<ImuSample> imu_samples;
    lidar_2_capture.done = !goal->use_second_lidar;
    bool imu_done = !goal->use_imu;

    auto record_start = std::chrono::steady_clock::now();

    auto cloud_callback = [&record_start, &goal](
                            PointCloudCapture & capture,
                            sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - record_start).count();
      append_cloud_sample(capture, *msg, elapsed, goal->acc_time_s);
    };

    auto imu_callback = [&imu_samples, &record_start, &goal](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - record_start).count();

      if (elapsed <= static_cast<double>(goal->imu_dur_s)) {
        imu_samples.push_back({
          msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z
        });
      }
    };

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto pc_sub_1 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      goal->pc_topic, sensor_qos,
      [&cloud_callback, &lidar_1_capture](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        cloud_callback(lidar_1_capture, std::move(msg));
      });
    RCLCPP_INFO(this->get_logger(), "Subscribed to lidar 1 topic: %s", goal->pc_topic.c_str());

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_2;
    if (goal->use_second_lidar) {
      pc_sub_2 = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        goal->pc_topic_2, sensor_qos,
        [&cloud_callback, &lidar_2_capture](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloud_callback(lidar_2_capture, std::move(msg));
        });
      RCLCPP_INFO(this->get_logger(), "Subscribed to lidar 2 topic: %s", goal->pc_topic_2.c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    if (goal->use_imu) {
      imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        goal->imu_topic, sensor_qos, imu_callback);
      RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: %s", goal->imu_topic.c_str());
    }

    // Create output directory
    const auto now_t = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now_t);
    std::tm tm_buf{};
    localtime_r(&tt, &tm_buf);
    std::ostringstream oss;
    std::string dir_ns = robot_ns_;
    if (!dir_ns.empty() && dir_ns.front() == '/') {
      dir_ns.erase(dir_ns.begin());
    }
    std::replace(dir_ns.begin(), dir_ns.end(), '/', '_');
    if (dir_ns.empty()) {
      dir_ns = "root";
    }
    oss << "/tmp/pcd_captures/" << dir_ns << "/" << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
    std::string output_dir = oss.str();

    std::error_code ec;
    std::filesystem::create_directories(output_dir, ec);
    if (ec) {
      result->success = false;
      result->error_msg = "Cannot create directory: " + ec.message();
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Writing capture output to %s", output_dir.c_str());

    result->output_dir = output_dir;

    // Recording loop
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - record_start).count();

      // Time-based completion
      if (!lidar_1_capture.done && goal->acc_time_s > 0.0f && elapsed >= static_cast<double>(goal->acc_time_s)) {
        lidar_1_capture.done = true;
      }
      if (!lidar_2_capture.done && goal->acc_time_s > 0.0f && elapsed >= static_cast<double>(goal->acc_time_s)) {
        lidar_2_capture.done = true;
      }
      if (!imu_done && goal->use_imu && elapsed >= static_cast<double>(goal->imu_dur_s)) {
        imu_done = true;
      }

      // Send feedback
      feedback->progress = 1.0f;
      if ((!lidar_1_capture.done || !lidar_2_capture.done) && goal->acc_time_s > 0.0f) {
        feedback->progress = static_cast<float>(
          std::min(elapsed / static_cast<double>(goal->acc_time_s), 1.0));
        char buf[192];
        if (goal->use_second_lidar) {
          std::snprintf(buf, sizeof(buf),
                        "Recording PC: %.1f / %.1f s [lidar1=%zu, lidar2=%zu clouds]",
                        std::min(elapsed, static_cast<double>(goal->acc_time_s)),
                        static_cast<double>(goal->acc_time_s),
                        lidar_1_capture.cloud_count, lidar_2_capture.cloud_count);
        } else {
          std::snprintf(buf, sizeof(buf), "Recording PC: %.1f / %.1f s [%zu clouds]",
                        std::min(elapsed, static_cast<double>(goal->acc_time_s)),
                        static_cast<double>(goal->acc_time_s), lidar_1_capture.cloud_count);
        }
        feedback->status = buf;
      } else if (goal->use_imu && !imu_done) {
        feedback->progress = static_cast<float>(
          std::min(elapsed / static_cast<double>(goal->imu_dur_s), 1.0));
        char buf[128];
        std::snprintf(buf, sizeof(buf), "PC done. IMU: %.1f / %.1f s [%zu samples]",
                      std::min(elapsed, static_cast<double>(goal->imu_dur_s)),
                      static_cast<double>(goal->imu_dur_s), imu_samples.size());
        feedback->status = buf;
      } else {
        feedback->status = "Done";
      }

      feedback->cloud_count = static_cast<uint32_t>(
        lidar_1_capture.cloud_count + lidar_2_capture.cloud_count);
      feedback->imu_samples_count = imu_samples.size();
      goal_handle->publish_feedback(feedback);

      if (lidar_1_capture.done && lidar_2_capture.done && imu_done) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Clean up subscriptions
    pc_sub_1.reset();
    pc_sub_2.reset();
    imu_sub.reset();

    if (goal_handle->is_canceling()) {
      result->error_msg = "Capture canceled by client";
      RCLCPP_INFO(this->get_logger(), "%s", result->error_msg.c_str());
      goal_handle->canceled(result);
      return;
    }

    // Save results
    if (!lidar_1_capture.accumulated_valid) {
      result->success = false;
      result->error_msg = "No pointcloud data received on lidar 1";
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }
    if (goal->use_second_lidar && !lidar_2_capture.accumulated_valid) {
      result->success = false;
      result->error_msg = "No pointcloud data received on lidar 2";
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }

    const std::string lidar_1_pcd_path = output_dir + "/cloud_lidar_1.pcd";
    const std::string lidar_2_pcd_path = output_dir + "/cloud_lidar_2.pcd";
    const std::string merged_pcd_path = output_dir + "/cloud_merged.pcd";
    const std::string legacy_pcd_path = output_dir + "/cloud.pcd";
    const std::string lidar_1_topic_path = output_dir + "/cloud_lidar_1.topic";
    const std::string lidar_2_topic_path = output_dir + "/cloud_lidar_2.topic";
    const std::string merged_topics_path = output_dir + "/cloud_merged.topics";
    const std::string gravity_path = output_dir + "/cloud_merged.gravity";
    const std::string legacy_gravity_path = output_dir + "/cloud.gravity";
    const std::string meta_path = output_dir + "/metadata.txt";

    size_t num_pts_lidar_1 = 0;
    if (!save_cloud_to_pcd(lidar_1_capture.accumulated, lidar_1_pcd_path, num_pts_lidar_1)) {
      result->success = false;
      result->error_msg = "Failed to write lidar 1 PCD file";
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }

    size_t num_pts_lidar_2 = 0;

    sensor_msgs::msg::PointCloud2 merged_cloud = lidar_1_capture.accumulated;
    if (goal->use_second_lidar) {
      if (!has_xyz_fields(lidar_2_capture.accumulated)) {
        result->success = false;
        result->error_msg = "Lidar 2 cloud is missing x/y/z fields required for merging";
        RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
        goal_handle->abort(result);
        return;
      }

      if (!save_cloud_to_pcd(lidar_2_capture.accumulated, lidar_2_pcd_path, num_pts_lidar_2)) {
        result->success = false;
        result->error_msg = "Failed to write lidar 2 PCD file";
        RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
        goal_handle->abort(result);
        return;
      }

      sensor_msgs::msg::PointCloud2 transformed_lidar_2 =
        transform_cloud_xyz(lidar_2_capture.accumulated, lidar_2_to_1);
      sensor_msgs::msg::PointCloud2 merged_cloud_out;
      pcl::concatenatePointCloud(merged_cloud, transformed_lidar_2, merged_cloud_out);
      merged_cloud = std::move(merged_cloud_out);
    }

    size_t num_pts_merged = 0;
    if (!save_cloud_to_pcd(merged_cloud, merged_pcd_path, num_pts_merged) ||
        !save_cloud_to_pcd(merged_cloud, legacy_pcd_path, num_pts_merged)) {
      result->success = false;
      result->error_msg = "Failed to write merged PCD file";
      RCLCPP_INFO(this->get_logger(), "Aborting capture: %s", result->error_msg.c_str());
      goal_handle->abort(result);
      return;
    }

    result->num_points = static_cast<uint32_t>(num_pts_merged);

    {
      std::ofstream lidar_1_topic_file(lidar_1_topic_path);
      if (lidar_1_topic_file.is_open()) {
        lidar_1_topic_file << goal->pc_topic << "\n";
      }
      if (goal->use_second_lidar) {
        std::ofstream lidar_2_topic_file(lidar_2_topic_path);
        if (lidar_2_topic_file.is_open()) {
          lidar_2_topic_file << goal->pc_topic_2 << "\n";
        }
      }
      std::ofstream merged_topics_file(merged_topics_path);
      if (merged_topics_file.is_open()) {
        merged_topics_file << goal->pc_topic << "\n";
        if (goal->use_second_lidar) {
          merged_topics_file << goal->pc_topic_2 << "\n";
        }
      }
    }

    // Save gravity (if IMU used)
    double ax = 0.0, ay = 0.0, az = 0.0;
    if (goal->use_imu) {
      if (!imu_samples.empty()) {
        for (const auto& s : imu_samples) {
          ax += s.x;
          ay += s.y;
          az += s.z;
        }
        const double n = static_cast<double>(imu_samples.size());
        ax /= n;
        ay /= n;
        az /= n;

        std::ofstream ofs(gravity_path);
        if (ofs.is_open()) {
          char gbuf[128];
          std::snprintf(gbuf, sizeof(gbuf), "%.9f %.9f %.9f\n", ax, ay, az);
          ofs << gbuf;
        }

        std::ofstream legacy_ofs(legacy_gravity_path);
        if (legacy_ofs.is_open()) {
          char gbuf[128];
          std::snprintf(gbuf, sizeof(gbuf), "%.9f %.9f %.9f\n", ax, ay, az);
          legacy_ofs << gbuf;
        }
      }
    }

    result->gravity_x = static_cast<float>(ax);
    result->gravity_y = static_cast<float>(ay);
    result->gravity_z = static_cast<float>(az);

    // Save metadata
    {
      const auto meta_now = std::chrono::system_clock::now();
      const std::time_t meta_tt = std::chrono::system_clock::to_time_t(meta_now);
      std::tm meta_tm{};
      localtime_r(&meta_tt, &meta_tm);
      std::ofstream mf(meta_path);
      if (mf.is_open()) {
        mf << "robot_ns:     " << robot_ns_ << "\n";
        mf << "timestamp:    " << std::put_time(&meta_tm, "%Y-%m-%dT%H:%M:%S") << "\n";
        mf << "pc_topic_1:   " << goal->pc_topic << "\n";
        mf << "pc_topic_2:   " << (goal->use_second_lidar ? goal->pc_topic_2 : "") << "\n";
        mf << "dual_lidar:   " << (goal->use_second_lidar ? "true" : "false") << "\n";
        mf << "acc_time_s:   " << goal->acc_time_s << "\n";
        mf << "clouds_1:     " << lidar_1_capture.cloud_count << "\n";
        mf << "clouds_2:     " << lidar_2_capture.cloud_count << "\n";
        mf << "points_1:     " << num_pts_lidar_1 << "\n";
        mf << "points_2:     " << num_pts_lidar_2 << "\n";
        mf << "num_points:   " << num_pts_merged << "\n";
        mf << "merged_pcd:   cloud_merged.pcd\n";
        mf << "lidar_1_pcd:  cloud_lidar_1.pcd\n";
        mf << "lidar_2_pcd:  " << (goal->use_second_lidar ? "cloud_lidar_2.pcd" : "") << "\n";
        if (goal->use_second_lidar) {
          mf << "lidar_1_to_2_transform:\n" << goal->lidar_1_to_2_transform << "\n";
        }
        if (goal->use_imu) {
          mf << "imu_topic:    " << goal->imu_topic << "\n";
          mf << "imu_dur_s:    " << goal->imu_dur_s << "\n";
          mf << "imu_samples:  " << imu_samples.size() << "\n";
          char gbuf[128];
          std::snprintf(gbuf, sizeof(gbuf), "%.6f %.6f %.6f", ax, ay, az);
          mf << "gravity_mps2: " << gbuf << "\n";
        }
      }
    }

    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Capture complete: merged=%zu points saved to %s",
                num_pts_merged, output_dir.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlaveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
