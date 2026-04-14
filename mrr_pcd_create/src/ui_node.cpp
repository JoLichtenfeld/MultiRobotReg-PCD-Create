#include <algorithm>
#include <chrono>
#include <cfloat>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "mrr_pcd_create_msgs/msg/robot_announcement.hpp"
#include "mrr_pcd_create_msgs/action/capture_pcd.hpp"

using CapturePcd = mrr_pcd_create_msgs::action::CapturePcd;
using GoalHandleCapturePcd = rclcpp_action::ClientGoalHandle<CapturePcd>;

// ─────────────────────────────────────────────────────────────────────────

struct RobotConfig {
  std::string robot_ns;
  std::string capture_action_name;
  rclcpp_action::Client<CapturePcd>::SharedPtr action_client;
  std::vector<std::string> pc_topics;
  std::vector<std::string> imu_topics;
  std::chrono::steady_clock::time_point last_announcement_time{};
  std::string last_output_dir;
  
  char pc_topic_buf[256] = "";
  char pc_topic_2_buf[256] = "";
  char imu_topic_buf[256] = "";
  char sync_host_buf[128] = "";
  char sync_user_buf[64] = "";
  char lidar_1_to_2_tf_buf[1024] =
    "1 0 0 0\n"
    "0 1 0 0\n"
    "0 0 1 0\n"
    "0 0 0 1";
  float acc_time = 1.0f;
  float imu_dur = 1.0f;
  bool use_imu = true;
  bool use_second_lidar = false;
  
  std::string status_msg;
  float progress = 0.0f;
  bool capturing = false;
  bool has_result = false;
  bool last_result_success = false;
};

struct LogEntry {
  std::chrono::system_clock::time_point timestamp;
  std::string message;
};

// ─────────────────────────────────────────────────────────────────────────

class MasterNode : public rclcpp::Node {
public:
  static constexpr double kInactiveTimeoutSec = 3.0;
  static std::string default_capture_action_for_robot(const std::string & robot_ns) {
    return robot_ns == "/" ? "/mrr_pcd_capture" : robot_ns + "/mrr_pcd_capture";
  }

  MasterNode() : Node("mrr_pcd_create_master") {
    // Subscriber to robot announcements
    announcement_sub_ = this->create_subscription<mrr_pcd_create_msgs::msg::RobotAnnouncement>(
      "/mrr_pcd_announcement", rclcpp::SystemDefaultsQoS(),
      std::bind(&MasterNode::on_announcement, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Master node started, listening for robots...");
  }

  std::map<std::string, RobotConfig> robots;
  std::vector<LogEntry> log_entries;
  char sync_local_root_buf[256] = "/tmp/mrr_pcd_sync";

private:
  rclcpp::Subscription<mrr_pcd_create_msgs::msg::RobotAnnouncement>::SharedPtr announcement_sub_;

  static std::string robot_ns_to_host_hint(const std::string & robot_ns) {
    std::string key = robot_ns;
    if (!key.empty() && key.front() == '/') {
      key.erase(key.begin());
    }
    if (key.empty()) {
      key = "robot";
    }
    std::replace(key.begin(), key.end(), '/', '-');
    return key;
  }

  static std::string shell_quote(const std::string & value) {
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('\'');
    for (char c : value) {
      if (c == '\'') {
        out += "'\\''";
      } else {
        out.push_back(c);
      }
    }
    out.push_back('\'');
    return out;
  }

  void on_announcement(const mrr_pcd_create_msgs::msg::RobotAnnouncement::SharedPtr msg) {
    auto it = robots.find(msg->robot_ns);
    if (it == robots.end()) {
      // New robot discovered
      RobotConfig cfg;
      cfg.robot_ns = msg->robot_ns;
      cfg.capture_action_name = msg->capture_action_name.empty() ?
        default_capture_action_for_robot(msg->robot_ns) : msg->capture_action_name;
      cfg.pc_topics = msg->available_pc_topics;
      cfg.imu_topics = msg->available_imu_topics;
      cfg.last_announcement_time = std::chrono::steady_clock::now();
      const std::string host_hint = robot_ns_to_host_hint(msg->robot_ns);
      std::strncpy(cfg.sync_host_buf, host_hint.c_str(), sizeof(cfg.sync_host_buf) - 1);
      
      // Auto-select first available topics
      if (!cfg.pc_topics.empty()) {
        std::strncpy(cfg.pc_topic_buf, cfg.pc_topics[0].c_str(), sizeof(cfg.pc_topic_buf) - 1);
      }
      if (cfg.pc_topics.size() > 1) {
        std::strncpy(cfg.pc_topic_2_buf, cfg.pc_topics[1].c_str(), sizeof(cfg.pc_topic_2_buf) - 1);
      } else if (!cfg.pc_topics.empty()) {
        std::strncpy(cfg.pc_topic_2_buf, cfg.pc_topics[0].c_str(), sizeof(cfg.pc_topic_2_buf) - 1);
      }
      if (!cfg.imu_topics.empty()) {
        std::strncpy(cfg.imu_topic_buf, cfg.imu_topics[0].c_str(), sizeof(cfg.imu_topic_buf) - 1);
      }
      
      robots[msg->robot_ns] = cfg;
      add_log("Discovered robot namespace: " + msg->robot_ns);
      RCLCPP_INFO(this->get_logger(),
                  "Discovered robot namespace: %s action=%s with %zu PC topics and %zu IMU topics",
                  msg->robot_ns.c_str(), cfg.capture_action_name.c_str(),
                  msg->available_pc_topics.size(), msg->available_imu_topics.size());
    } else {
      // Update existing robot's topics
      it->second.capture_action_name = msg->capture_action_name.empty() ?
        default_capture_action_for_robot(msg->robot_ns) : msg->capture_action_name;
      it->second.pc_topics = msg->available_pc_topics;
      it->second.imu_topics = msg->available_imu_topics;
      it->second.last_announcement_time = std::chrono::steady_clock::now();
    }
  }

public:
  bool is_active(const RobotConfig & cfg) const {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - cfg.last_announcement_time).count();
    return elapsed <= kInactiveTimeoutSec;
  }

  double seconds_since_last_announcement(const RobotConfig & cfg) const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - cfg.last_announcement_time).count();
  }

  void add_log(const std::string& msg) {
    LogEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.message = msg;
    log_entries.push_back(entry);
    
    // Keep log size manageable (max 100 entries)
    if (log_entries.size() > 100) {
      log_entries.erase(log_entries.begin());
    }
  }

  void trigger_capture_all() {
    if (robots.empty()) {
      add_log("ERROR: No robots discovered!");
      return;
    }

    add_log("Triggering capture on " + std::to_string(robots.size()) + " robot(s)...");

    for (auto& [robot_ns, cfg] : robots) {
      if (!is_active(cfg)) {
        cfg.status_msg = "INACTIVE: no announcement for >3s";
        add_log("  [" + robot_ns + "] SKIPPED: inactive (>3s without announcement)");
        continue;
      }

      if (cfg.pc_topic_buf[0] == '\0') {
        cfg.status_msg = "ERROR: No PC topic selected";
        add_log("  [" + robot_ns + "] ERROR: No PC topic selected");
        continue;
      }

      if (cfg.use_second_lidar && cfg.pc_topic_2_buf[0] == '\0') {
        cfg.status_msg = "ERROR: No lidar 2 topic selected";
        add_log("  [" + robot_ns + "] ERROR: No lidar 2 topic selected");
        continue;
      }

      trigger_capture(robot_ns, cfg);
    }
  }

  bool sync_robot_last_output(const std::string & robot_ns, RobotConfig & cfg) {
    if (cfg.last_output_dir.empty()) {
      add_log("  [" + robot_ns + "] SKIPPED: no captured output directory yet");
      return false;
    }

    const std::string host(cfg.sync_host_buf);
    if (host.empty()) {
      add_log("  [" + robot_ns + "] ERROR: sync host is empty");
      cfg.status_msg = "ERROR: Sync host is empty";
      return false;
    }

    const std::string user(cfg.sync_user_buf);
    const std::string remote_spec = user.empty() ? host : (user + "@" + host);
    const std::filesystem::path remote_path(cfg.last_output_dir);
    const std::string leaf = remote_path.filename().string().empty() ? "capture" : remote_path.filename().string();
    const std::string local_root(sync_local_root_buf);
    const std::string robot_leaf = robot_ns_to_host_hint(robot_ns);
    const std::filesystem::path local_target = std::filesystem::path(local_root) / robot_leaf / leaf;

    std::error_code ec;
    std::filesystem::create_directories(local_target.parent_path(), ec);
    if (ec) {
      add_log("  [" + robot_ns + "] ERROR: cannot create local sync directory: " + ec.message());
      cfg.status_msg = "ERROR: cannot create local sync directory";
      return false;
    }

    std::ostringstream cmd;
    cmd << "rsync -az "
        << shell_quote(remote_spec + ":" + cfg.last_output_dir + "/") << " "
        << shell_quote(local_target.string() + "/");

    add_log("  [" + robot_ns + "] Syncing " + cfg.last_output_dir + " -> " + local_target.string());
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
      cfg.status_msg = "ERROR: Sync failed";
      add_log("  [" + robot_ns + "] ERROR: rsync failed (code " + std::to_string(rc) + ")");
      return false;
    }

    cfg.status_msg = "Synced to " + local_target.string();
    add_log("  [" + robot_ns + "] Sync complete: " + local_target.string());
    return true;
  }

  void sync_all_last_outputs() {
    if (robots.empty()) {
      add_log("Sync skipped: no robots discovered");
      return;
    }

    add_log("Syncing latest captures from all robots...");
    std::size_t ok_count = 0;
    for (auto & [robot_ns, cfg] : robots) {
      if (sync_robot_last_output(robot_ns, cfg)) {
        ++ok_count;
      }
    }
    add_log("Sync summary: " + std::to_string(ok_count) + "/" + std::to_string(robots.size()) +
            " robot outputs copied");
  }

private:
  void trigger_capture(const std::string& robot_ns, RobotConfig& cfg) {
    // Create action client
    cfg.action_client = rclcpp_action::create_client<CapturePcd>(
      this, cfg.capture_action_name);
    auto action_client = cfg.action_client;

    add_log("  [" + robot_ns + "] Using action endpoint: " + cfg.capture_action_name);

    if (!action_client->wait_for_action_server(std::chrono::seconds(2))) {
      cfg.status_msg = "Server not responding";
      cfg.has_result = true;
      cfg.last_result_success = false;
      add_log("  [" + robot_ns + "] ERROR: Action server not responding at " +
              cfg.capture_action_name);
      return;
    }

    auto goal = CapturePcd::Goal();
    goal.pc_topic = std::string(cfg.pc_topic_buf);
    goal.pc_topic_2 = cfg.use_second_lidar ? std::string(cfg.pc_topic_2_buf) : std::string();
    goal.imu_topic = std::string(cfg.imu_topic_buf);
    goal.acc_time_s = cfg.acc_time;
    goal.imu_dur_s = cfg.imu_dur;
    goal.use_imu = cfg.use_imu;
    goal.use_second_lidar = cfg.use_second_lidar;
    goal.lidar_1_to_2_transform =
      cfg.use_second_lidar ? std::string(cfg.lidar_1_to_2_tf_buf) : std::string();

    cfg.capturing = true;
    cfg.has_result = false;
    cfg.last_result_success = false;
    cfg.progress = 0.0f;
    cfg.status_msg = "Capturing...";

    auto send_goal_options = rclcpp_action::Client<CapturePcd>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, robot_ns, &cfg](const GoalHandleCapturePcd::SharedPtr& handle) {
        if (!handle) {
          cfg.capturing = false;
          cfg.has_result = true;
          cfg.last_result_success = false;
          cfg.status_msg = "Goal rejected";
          add_log("  [" + robot_ns + "] ERROR: Goal rejected");
        }
      };

    send_goal_options.feedback_callback =
      [this, robot_ns, &cfg](
        GoalHandleCapturePcd::SharedPtr,
        const std::shared_ptr<const CapturePcd::Feedback>& feedback) {
        (void)robot_ns;
        cfg.progress = feedback->progress;
        cfg.status_msg = feedback->status;
      };

    send_goal_options.result_callback =
      [this, robot_ns, &cfg](const GoalHandleCapturePcd::WrappedResult& result) {
        cfg.capturing = false;
        cfg.has_result = true;
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          cfg.last_result_success = true;
          cfg.last_output_dir = result.result->output_dir;
          cfg.status_msg = "Done: " + std::to_string(result.result->num_points) +
                           " merged points | " + result.result->output_dir;
          add_log("  [" + robot_ns + "] SUCCESS: " + std::to_string(result.result->num_points) + 
                  " merged points saved to " + result.result->output_dir);
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          cfg.last_result_success = false;
          cfg.status_msg = "CANCELED";
          add_log("  [" + robot_ns + "] CANCELED");
        } else {
          cfg.last_result_success = false;
          const std::string error_msg = result.result ? result.result->error_msg : "Unknown action error";
          cfg.status_msg = "FAILED: " + error_msg;
          add_log("  [" + robot_ns + "] ERROR: " + error_msg);
        }
      };

    action_client->async_send_goal(goal, send_goal_options);
    add_log("  [" + robot_ns + "] Capture triggered");
  }
};

// ─────────────────────────────────────────────────────────────────────────

static void topic_combo(const char* label,
                        const char* popup_id,
                        char* buf,
                        std::size_t buf_size,
                        const std::vector<std::string>& topics) {
  const float btn_w = ImGui::GetFrameHeight();
  const float inner = ImGui::GetStyle().ItemInnerSpacing.x;
  const float field_w = ImGui::CalcItemWidth() - btn_w - inner;

  const ImVec2 field_pos = ImGui::GetCursorScreenPos();

  ImGui::SetNextItemWidth(field_w);
  std::string input_id = std::string("##input_") + popup_id;
  ImGui::InputText(input_id.c_str(), buf, buf_size);
  ImGui::SameLine(0.0f, inner);
  std::string btn_id = std::string("##btn_") + popup_id;
  if (ImGui::ArrowButton(btn_id.c_str(), ImGuiDir_Down)) {
    const float popup_w = field_w + btn_w + inner;
    ImGui::SetNextWindowPos(ImVec2(field_pos.x, field_pos.y + ImGui::GetFrameHeight()),
                            ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(popup_w, 0.0f), ImGuiCond_Always);
    ImGui::OpenPopup(popup_id);
  }
  ImGui::SameLine(0.0f, inner);
  ImGui::TextUnformatted(label);

  if (ImGui::BeginPopup(popup_id)) {
    bool any = false;
    for (const auto& t : topics) {
      any = true;
      if (ImGui::Selectable(t.c_str())) {
        std::strncpy(buf, t.c_str(), buf_size - 1);
        buf[buf_size - 1] = '\0';
      }
    }
    if (!any)
      ImGui::TextDisabled(topics.empty() ? "(no topics discovered yet)" : "(no match)");
    ImGui::EndPopup();
  }
}

static void glfw_error_cb(int, const char* desc) {
  std::fprintf(stderr, "GLFW error: %s\n", desc);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MasterNode>();

  glfwSetErrorCallback(glfw_error_cb);
  if (!glfwInit()) return 1;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  GLFWwindow* win = glfwCreateWindow(900, 700, "mrr_pcd_create - Master UI", nullptr, nullptr);
  if (!win) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 330");
  ImGui::StyleColorsDark();

  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();
    rclcpp::spin_some(node);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("##main", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

    // ── Robot configurations (top section) ──────────────────────────────
    ImGui::Text("Connected Robots: %zu", node->robots.size());
    ImGui::Separator();

    if (node->robots.empty()) {
      ImGui::TextUnformatted("Waiting for robots to announce...");
    } else {
      std::size_t active_count = 0;
      for (const auto& [_, cfg] : node->robots) {
        if (node->is_active(cfg)) {
          ++active_count;
        }
      }
      ImGui::Text("Active: %zu  Inactive: %zu", active_count, node->robots.size() - active_count);

      int robot_idx = 0;
      for (auto& [robot_ns, cfg] : node->robots) {
        ImGui::PushID(robot_idx);
        const bool active = node->is_active(cfg);
        const double inactive_s = node->seconds_since_last_announcement(cfg);
        
        // Robot header
        if (ImGui::CollapsingHeader(("Robot NS: " + robot_ns).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Indent();

          if (active) {
            ImGui::TextColored(ImVec4(0.35f, 1.0f, 0.35f, 1.0f), "State: ACTIVE");
          } else {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f),
                               "State: INACTIVE (last seen %.1f s ago)", inactive_s);
          }

          if (!active) {
            ImGui::BeginDisabled();
          }
          
          topic_combo("Lidar 1 topic", ("##pc_1_" + robot_ns).c_str(),
                      cfg.pc_topic_buf, sizeof(cfg.pc_topic_buf),
                      cfg.pc_topics);

          ImGui::Checkbox(("Use second lidar##" + robot_ns).c_str(), &cfg.use_second_lidar);
          if (cfg.use_second_lidar) {
            topic_combo("Lidar 2 topic", ("##pc_2_" + robot_ns).c_str(),
                        cfg.pc_topic_2_buf, sizeof(cfg.pc_topic_2_buf),
                        cfg.pc_topics);
            ImGui::TextUnformatted("Transform from lidar 1 to lidar 2 (4x4)");
            ImGui::InputTextMultiline(("##tf_" + robot_ns).c_str(),
                                      cfg.lidar_1_to_2_tf_buf,
                                      sizeof(cfg.lidar_1_to_2_tf_buf),
                                      ImVec2(-FLT_MIN, ImGui::GetTextLineHeight() * 5.5f));
            ImGui::TextDisabled("Enter 16 values. Spaces, commas, semicolons, and newlines are accepted.");
          }

          ImGui::Checkbox(("Use IMU##" + robot_ns).c_str(), &cfg.use_imu);
          if (cfg.use_imu) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            topic_combo("IMU topic", ("##imu_" + robot_ns).c_str(),
                        cfg.imu_topic_buf, sizeof(cfg.imu_topic_buf),
                        cfg.imu_topics);
          }

          ImGui::InputFloat(("Acc. time (s)##" + robot_ns).c_str(), &cfg.acc_time, 0.5f, 1.0f, "%.1f");
          if (cfg.acc_time < 0.0f) cfg.acc_time = 0.0f;

          if (cfg.use_imu) {
            ImGui::InputFloat(("IMU duration (s)##" + robot_ns).c_str(), &cfg.imu_dur, 0.5f, 1.0f, "%.1f");
            if (cfg.imu_dur < 0.1f) cfg.imu_dur = 0.1f;
          }

          // Status
          if (cfg.capturing) {
            ImGui::ProgressBar(cfg.progress, ImVec2(-1.0f, 0.0f), "");
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.35f, 1.0f), "CAPTURING: %s", cfg.status_msg.c_str());
          } else if (cfg.has_result && cfg.last_result_success) {
            ImGui::TextColored(ImVec4(0.30f, 1.0f, 0.30f, 1.0f), "SUCCESS: %s", cfg.status_msg.c_str());
          } else if (cfg.has_result) {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", cfg.status_msg.c_str());
          } else {
            ImGui::TextWrapped("%s", cfg.status_msg.c_str());
          }
          if (!cfg.last_output_dir.empty()) {
            ImGui::TextWrapped("Last output: %s", cfg.last_output_dir.c_str());
          }

          ImGui::InputText(("Sync host##" + robot_ns).c_str(),
                           cfg.sync_host_buf, sizeof(cfg.sync_host_buf));
          ImGui::InputText(("Sync user (optional)##" + robot_ns).c_str(),
                           cfg.sync_user_buf, sizeof(cfg.sync_user_buf));

          if (cfg.capturing || cfg.last_output_dir.empty()) {
            ImGui::BeginDisabled();
          }
          if (ImGui::Button(("Copy/Sync Last Dump##" + robot_ns).c_str())) {
            node->sync_robot_last_output(robot_ns, cfg);
          }
          if (cfg.capturing || cfg.last_output_dir.empty()) {
            ImGui::EndDisabled();
          }

          if (!active) {
            ImGui::EndDisabled();
          }

          ImGui::Unindent();
        }

        ImGui::PopID();
        robot_idx++;
      }
    }

    ImGui::Separator();

    // ── Big green capture button ────────────────────────────────────────
    ImVec2 button_size(ImGui::GetContentRegionAvail().x, 50.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.8f, 0.2f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 1.0f, 0.3f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.6f, 0.15f, 1.0f));

    bool any_capturing = false;
    for (auto& [_, cfg] : node->robots) {
      if (cfg.capturing) {
        any_capturing = true;
        break;
      }
    }

    bool any_active = false;
    for (const auto& [_, cfg] : node->robots) {
      if (node->is_active(cfg)) {
        any_active = true;
        break;
      }
    }

    if (any_capturing || !any_active) ImGui::BeginDisabled();
    if (ImGui::Button("CAPTURE ALL", button_size)) {
      node->trigger_capture_all();
    }
    if (any_capturing || !any_active) ImGui::EndDisabled();

    ImGui::InputText("Sync local root", node->sync_local_root_buf, sizeof(node->sync_local_root_buf));
    if (any_capturing) ImGui::BeginDisabled();
    if (ImGui::Button("COPY/SYNC ALL", button_size)) {
      node->sync_all_last_outputs();
    }
    if (any_capturing) ImGui::EndDisabled();

    ImGui::PopStyleColor(3);

    ImGui::Separator();

    // ── Log box ────────────────────────────────────────────────────────
    ImGui::Text("Event Log:");
    ImGui::BeginChild("##log", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar);
    for (const auto& entry : node->log_entries) {
      const auto t = std::chrono::system_clock::to_time_t(entry.timestamp);
      std::tm tm_buf{};
      localtime_r(&t, &tm_buf);
      char time_str[32];
      std::strftime(time_str, sizeof(time_str), "%H:%M:%S", &tm_buf);
      ImGui::TextUnformatted(time_str);
      ImGui::SameLine();
      ImGui::TextWrapped("%s", entry.message.c_str());
    }
    ImGui::EndChild();

    ImGui::End();

    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(win, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(win);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(win);
  glfwTerminate();

  rclcpp::shutdown();
  return 0;
}
