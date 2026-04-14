#include <algorithm>
#include <chrono>
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

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "mrr_pcd_create_msgs/msg/robot_announcement.hpp"
#include "mrr_pcd_create_msgs/action/capture_pcd.hpp"

enum class State { IDLE, RECORDING, DONE, ERR };

struct ImuSample {
  double x, y, z;
};

using CapturePcd = mrr_pcd_create_msgs::action::CapturePcd;
using GoalHandleCapturePcd = rclcpp_action::ServerGoalHandle<CapturePcd>;

// ---------------------------------------------------------------------------

class SlaveNode : public rclcpp::Node {
public:
  SlaveNode() : Node("slave_node") {
    // Use robot namespace as robot identity.
    this->declare_parameter<std::string>("robot_ns", "/robot_0");
    robot_ns_ = normalize_namespace(this->get_parameter("robot_ns").as_string());

    // Publisher for robot announcements
    announcement_pub_ = this->create_publisher<mrr_pcd_create_msgs::msg::RobotAnnouncement>(
      "/mrr_pcd_announcement", rclcpp::SystemDefaultsQoS());

    // Action server
    action_server_ = rclcpp_action::create_server<CapturePcd>(
      this,
      "/mrr_pcd_capture",
      std::bind(&SlaveNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SlaveNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&SlaveNode::handle_accepted, this, std::placeholders::_1));

    // Timer for periodic announcements (every 2 seconds)
    announce_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&SlaveNode::publish_announcement, this));

    RCLCPP_INFO(this->get_logger(), "Slave node for namespace '%s' started", robot_ns_.c_str());
  }

private:
  std::string robot_ns_;
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

  // ---------- Announcement ----------
  void publish_announcement() {
    auto ann = mrr_pcd_create_msgs::msg::RobotAnnouncement();
    ann.robot_ns = robot_ns_;

    // Discover available topics
    for (const auto& [topic, types] : this->get_topic_names_and_types()) {
      if (!topic_in_robot_namespace(topic)) {
        continue;
      }
      for (const auto& t : types) {
        if (t == "sensor_msgs/msg/PointCloud2") {
          ann.available_pc_topics.push_back(topic);
        }
        if (t == "sensor_msgs/msg/Imu") {
          ann.available_imu_topics.push_back(topic);
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
    RCLCPP_INFO(this->get_logger(), "Received capture goal: PC topic=%s, IMU topic=%s, acc_time=%.1f",
                goal->pc_topic.c_str(), goal->imu_topic.c_str(), goal->acc_time_s);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCapturePcd> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCapturePcd> goal_handle) {
    // Create a new thread to execute the goal
    std::thread{std::bind(&SlaveNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCapturePcd> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CapturePcd::Feedback>();
    auto result = std::make_shared<CapturePcd::Result>();

    result->robot_ns = robot_ns_;
    result->success = false;

    // Setup recording
    State state = State::RECORDING;
    sensor_msgs::msg::PointCloud2 accumulated_;
    bool accumulated_valid = false;
    size_t cloud_count = 0;
    std::vector<ImuSample> imu_samples;
    bool pc_done = false;
    bool imu_done = !goal->use_imu;

    auto record_start = std::chrono::steady_clock::now();

    // Lambda for subscription callbacks
    auto cloud_callback = [&accumulated_, &accumulated_valid, &cloud_count, &pc_done,
                           &record_start, &goal](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      if (pc_done) return;

      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - record_start).count();

      if (goal->acc_time_s <= 0.0f) {
        accumulated_ = *msg;
        accumulated_valid = true;
        cloud_count = 1;
        pc_done = true;
      } else {
        if (!accumulated_valid) {
          accumulated_ = *msg;
          accumulated_valid = true;
        } else {
          sensor_msgs::msg::PointCloud2 merged;
          pcl::concatenatePointCloud(accumulated_, *msg, merged);
          accumulated_ = std::move(merged);
        }
        ++cloud_count;

        if (elapsed >= static_cast<double>(goal->acc_time_s)) {
          pc_done = true;
        }
      }
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
    auto pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      goal->pc_topic, sensor_qos, cloud_callback);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    if (goal->use_imu) {
      imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        goal->imu_topic, sensor_qos, imu_callback);
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
      goal_handle->abort(result);
      return;
    }

    result->output_dir = output_dir;

    // Recording loop
    while (rclcpp::ok() && !goal_handle->is_canceling()) {
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - record_start).count();

      // Time-based completion
      if (!pc_done && goal->acc_time_s > 0.0f && elapsed >= static_cast<double>(goal->acc_time_s)) {
        pc_done = true;
      }
      if (!imu_done && goal->use_imu && elapsed >= static_cast<double>(goal->imu_dur_s)) {
        imu_done = true;
      }

      // Send feedback
      feedback->progress = 1.0f;
      if (!pc_done && goal->acc_time_s > 0.0f) {
        feedback->progress = static_cast<float>(
          std::min(elapsed / static_cast<double>(goal->acc_time_s), 1.0));
        char buf[128];
        std::snprintf(buf, sizeof(buf), "Recording PC: %.1f / %.1f s [%zu clouds]",
                      std::min(elapsed, static_cast<double>(goal->acc_time_s)),
                      static_cast<double>(goal->acc_time_s), cloud_count);
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

      feedback->cloud_count = cloud_count;
      feedback->imu_samples_count = imu_samples.size();
      goal_handle->publish_feedback(feedback);

      if (pc_done && imu_done) {
        break;
      }

      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Clean up subscriptions
    pc_sub.reset();
    imu_sub.reset();

    // Save results
    if (!accumulated_valid) {
      result->success = false;
      result->error_msg = "No pointcloud data received";
      goal_handle->abort(result);
      return;
    }

    const std::string pcd_path = output_dir + "/cloud.pcd";
    const std::string gravity_path = output_dir + "/cloud.gravity";
    const std::string meta_path = output_dir + "/metadata.txt";

    // Save PCD
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(accumulated_, pcl_cloud);
    pcl::PCDWriter writer;
    if (writer.writeBinary(pcd_path, pcl_cloud) != 0) {
      result->success = false;
      result->error_msg = "Failed to write PCD file";
      goal_handle->abort(result);
      return;
    }

    const size_t num_pts = static_cast<size_t>(pcl_cloud.width) * static_cast<size_t>(pcl_cloud.height);
    result->num_points = num_pts;

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
        mf << "pc_topic:     " << goal->pc_topic << "\n";
        mf << "acc_time_s:   " << goal->acc_time_s << "\n";
        mf << "num_points:   " << num_pts << "\n";
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

    RCLCPP_INFO(this->get_logger(), "Capture complete: %zu points saved to %s",
                num_pts, output_dir.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlaveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
