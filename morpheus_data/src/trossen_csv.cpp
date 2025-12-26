// joy_dual_logger.cpp
#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "interbotix_xs_msgs/ArmJoy.h"

// ---- PS4 mapping (extend if needed) ----
static const std::map<std::string,int> ps4 = {
  {"GRIPPER_PWM_DEC", 0},  // buttons start here
  {"GRIPPER_OPEN",    1},
  {"GRIPPER_PWM_INC", 2},
  {"GRIPPER_CLOSE",   3},
  {"EE_Y_INC",        4},
  {"EE_Y_DEC",        5},
  {"WAIST_CCW",       6},
  {"WAIST_CW",        7},
  {"SLEEP_POSE",      8},
  {"HOME_POSE",       9},
  {"TORQUE_ENABLE",   10},
  {"FLIP_EE_X",       11},
  {"FLIP_EE_ROLL",    12},
  // axes start here
  {"EE_X",            0},
  {"EE_Z",            1},
  {"EE_ROLL",         3},
  {"EE_PITCH",        4},
  {"SPEED_TYPE",      6},
  {"SPEED",           7}
};

class JoyDualLogger {
public:
  JoyDualLogger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
    // ------------ Parameters ------------
    pnh_.param<std::string>("basename_topic", basename_topic_, std::string("/UCDsession/csv_basename"));
    pnh_.param<std::string>("joy_topic", joy_topic_, std::string("/vx300s/joy"));
    pnh_.param<std::string>("armjoy_topic", armjoy_topic_, std::string("/vx300s/commands/joy_processed"));
    pnh_.param<std::string>("base_dir", base_dir_, std::string("/root/catkin_ws/src/morpheus_data/data/"));
    pnh_.param<std::string>("controller", controller_type_, std::string("ps4"));
    pnh_.param<double>("on_threshold", on_thr_, 0.75);        // single threshold
    pnh_.param<bool>("write_combined", write_combined_, true);
    pnh_.param<bool>("append", append_, false);               // per trial, we truncate by default
    pnh_.param<bool>("flush_each_row", flush_each_row_, false);
    pnh_.param<double>("log_rate_hz", log_rate_hz_, 10.0);    // 10 Hz => 0.1s

    // Controller mapping
    if (controller_type_ == "ps4") {
      cntlr_ = ps4;
    } else {
      ROS_WARN("Unknown controller '%s'; defaulting to ps4 mapping.", controller_type_.c_str());
      cntlr_ = ps4;
    }

    // ------------ Subscriptions ------------
    joy_sub_       = nh_.subscribe(joy_topic_, 200, &JoyDualLogger::joyCb, this);
    armjoy_sub_    = nh_.subscribe(armjoy_topic_, 100, &JoyDualLogger::armjoyCb, this);
    csv_base_sub_  = nh_.subscribe(basename_topic_, 1, &JoyDualLogger::csvBaseCb, this);

    // ------------ Fixed-rate timer for logging ------------
    const double safe_rate = std::max(1e-6, log_rate_hz_);
    timer_ = nh_.createTimer(ros::Duration(1.0 / safe_rate),
                             &JoyDualLogger::timerCb, this);

    ROS_INFO_STREAM("JoyDualLogger starting. base_dir=" << base_dir_
                    << " controller=" << controller_type_
                    << " on_threshold=" << on_thr_
                    << " write_combined=" << (write_combined_ ? "true" : "false")
                    << " log_rate_hz=" << log_rate_hz_);
  }

  ~JoyDualLogger() {
    closeAllCsv();
  }

private:
  // ========================= CSV rotation / lifecycle =========================
  void csvBaseCb(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    const std::string s = msg->data;
    if (s.empty()) {
      // Trial finished: close and idle
      closeAllCsv();
      joy_header_written_ = armjoy_header_written_ = combined_header_written_ = false;
      active_base_.clear();
      have_t0_ = false;  // reset session origin
      ROS_INFO("[JoyDualLogger] Session ended closed CSVs; waiting for next basename.");
    } else {
      // New trial: rotate files
      active_base_ = s;
      reopenAllCsvWithBase(active_base_);
      session_t0_ = ros::Time::now();   // session origin for t_rel
      have_t0_ = true;
      ROS_INFO_STREAM("[JoyDualLogger] Session started base=" << active_base_
                      << " t0=" << session_t0_.toSec());
    }
  }

  void reopenAllCsvWithBase(const std::string& base) {
    closeAllCsv();

    // Ensure dir exists
    try {
      std::filesystem::create_directories(base_dir_);
    } catch (...) {
      ROS_WARN_STREAM("Could not ensure directory exists: " << base_dir_);
    }

    // Reset headers so we re-emit for the new files
    joy_header_written_ = armjoy_header_written_ = combined_header_written_ = false;

    // Build paths
    const std::string base_full = joinPath(base_dir_, base);
    joy_csv_path_      = base_full + "_joy_raw.csv";
    armjoy_csv_path_   = base_full + "_armjoy_cmd.csv";
    combined_csv_path_ = base_full + "_joy_and_cmd.csv";
    cal_csv_path_      = base_full + "_cal_joy.csv";

    // (Re)open
    openCsv(joy_csv_, joy_csv_path_, /*append=*/false);
    openCsv(armjoy_csv_, armjoy_csv_path_, /*append=*/false);
    openCsv(cal_csv_,  cal_csv_path_,  /*append=*/false);
    if (write_combined_) openCsv(combined_csv_, combined_csv_path_, /*append=*/false);

    ROS_INFO_STREAM("[JoyDualLogger] Writing to:"
                    << "\n  joy_csv=" << joy_csv_path_
                    << "\n  armjoy_csv=" << armjoy_csv_path_
                    << "\n  cal_csv=" << cal_csv_path_
                    << "\n  combined_csv=" << (write_combined_ ? combined_csv_path_ : "(disabled)"));
  }

  void closeAllCsv() {
    if (joy_csv_.is_open()) joy_csv_.close();
    if (armjoy_csv_.is_open()) armjoy_csv_.close();
    if (combined_csv_.is_open()) combined_csv_.close();
    if (cal_csv_.is_open()) cal_csv_.close();
  }

  static std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/' || a.back() == '\\') return a + b;
    return a + "/" + b;
  }

  void openCsv(std::ofstream& f, const std::string& path, bool append) {
    std::ios_base::openmode mode = std::ios::out | (append ? std::ios::app : std::ios::trunc);
    f.open(path.c_str(), mode);
    if (!f) ROS_ERROR_STREAM("Failed to open CSV: " << path);
  }

  // ========================= Joy → ArmJoy derivation =========================
  interbotix_xs_msgs::ArmJoy deriveArmJoy(const sensor_msgs::Joy& j) {
    interbotix_xs_msgs::ArmJoy m;
    auto idx = [&](const char* key)->int { return cntlr_.at(key); };

    // Axes -> thresholded commands
    if (!j.axes.empty()) {
      const double ax = j.axes.at(idx("EE_X"));
      if      (ax >= on_thr_) m.ee_x_cmd = interbotix_xs_msgs::ArmJoy::EE_X_INC;
      else if (ax <= -on_thr_) m.ee_x_cmd = interbotix_xs_msgs::ArmJoy::EE_X_DEC;

      const double az = j.axes.at(idx("EE_Z"));
      if      (az >= on_thr_) m.ee_z_cmd = interbotix_xs_msgs::ArmJoy::EE_Z_INC;
      else if (az <= -on_thr_) m.ee_z_cmd = interbotix_xs_msgs::ArmJoy::EE_Z_DEC;

      const double ar = j.axes.at(idx("EE_ROLL"));
      if      (ar >= on_thr_) m.ee_roll_cmd = interbotix_xs_msgs::ArmJoy::EE_ROLL_CW;
      else if (ar <= -on_thr_) m.ee_roll_cmd = interbotix_xs_msgs::ArmJoy::EE_ROLL_CCW;

      const double ap = j.axes.at(idx("EE_PITCH"));
      if      (ap >= on_thr_) m.ee_pitch_cmd = interbotix_xs_msgs::ArmJoy::EE_PITCH_UP;
      else if (ap <= -on_thr_) m.ee_pitch_cmd = interbotix_xs_msgs::ArmJoy::EE_PITCH_DOWN;
    }

    // Buttons
    auto getBtn = [&](const char* k)->int {
      int i = cntlr_.at(k);
      return (i >= 0 && (size_t)i < j.buttons.size()) ? j.buttons[i] : 0;
    };

    if (getBtn("EE_Y_INC") == 1) m.ee_y_cmd = interbotix_xs_msgs::ArmJoy::EE_Y_INC;
    else if (getBtn("EE_Y_DEC") == 1) m.ee_y_cmd = interbotix_xs_msgs::ArmJoy::EE_Y_DEC;

    if (getBtn("WAIST_CCW") == 1)      m.waist_cmd = interbotix_xs_msgs::ArmJoy::WAIST_CCW;
    else if (getBtn("WAIST_CW") == 1)  m.waist_cmd = interbotix_xs_msgs::ArmJoy::WAIST_CW;

    if (getBtn("GRIPPER_CLOSE") == 1)  m.gripper_cmd = interbotix_xs_msgs::ArmJoy::GRIPPER_CLOSE;
    else if (getBtn("GRIPPER_OPEN") == 1) m.gripper_cmd = interbotix_xs_msgs::ArmJoy::GRIPPER_OPEN;

    if (getBtn("HOME_POSE") == 1)      m.pose_cmd = interbotix_xs_msgs::ArmJoy::HOME_POSE;
    else if (getBtn("SLEEP_POSE") == 1)  m.pose_cmd = interbotix_xs_msgs::ArmJoy::SLEEP_POSE;

    return m;
  }

  // ========================= ROS Callbacks =========================
  // Note: no file writes here; we cache the latest state only.
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    last_joy_ = *msg;
    have_joy_ = true;
  }

  void armjoyCb(const interbotix_xs_msgs::ArmJoy::ConstPtr& msg) {
    // Optional pass-through caching (combined uses derived cmd by default)
    std::lock_guard<std::mutex> lk(mtx_);
    last_armjoy_ = *msg;
    have_armjoy_ = true;
  }

  // ========================= Timer: periodic writers (10 Hz default) =========================
  void timerCb(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double t_now = now.toSec();
    std::lock_guard<std::mutex> lk(mtx_);

    // Only log during an active session and if we have a Joy sample and t0
    if (active_base_.empty() || !have_joy_ || !have_t0_) return;

    const double t_rel = (now - session_t0_).toSec();

    // Ensure headers now that we know vector sizes
    if (!joy_header_written_) {
      writeJoyHeader();
      joy_header_written_ = true;
    }
    if (write_combined_ && !combined_header_written_) {
      writeCombinedHeader();
      combined_header_written_ = true;
    }

    // Log joy_raw at fixed rate
    writeJoyRow(t_now, t_rel, last_joy_);

    // Keep a cal snapshot at the same rate (useful for threshold tuning)
    writeCalRow(t_now, t_rel, last_joy_);

    // Derive ArmJoy from latest Joy for combined
    interbotix_xs_msgs::ArmJoy derived = deriveArmJoy(last_joy_);
    last_armjoy_  = derived;
    have_armjoy_  = true;

    // Log combined at fixed rate (joy_and_cmd)
    if (write_combined_) {
      writeCombinedRow(t_now, t_rel, "timer");
    }

    // If you also want armjoy-only CSV at 10 Hz, uncomment:
    // if (!armjoy_header_written_) { writeArmJoyHeader(); armjoy_header_written_ = true; }
    // writeArmJoyRow(t_now, derived);
  }

  // ========================= CSV writers =========================
  void writeJoyHeader() {
    if (!joy_csv_) return;
    joy_csv_ << "recv_time,t_rel,joy_header_sec,joy_header_nsec,seq";
    for (size_t i = 0; i < last_joy_.axes.size(); ++i)    joy_csv_ << ",axes_"   << i;
    for (size_t j = 0; j < last_joy_.buttons.size(); ++j) joy_csv_ << ",button_" << j;
    joy_csv_ << "\n";
    if (flush_each_row_) joy_csv_.flush();

    if (cal_csv_) {
      cal_csv_ << "recv_time,t_rel,axes_EE_X,axes_EE_Z,axes_EE_ROLL,axes_EE_PITCH\n";
      if (flush_each_row_) cal_csv_.flush();
    }
  }

  void writeArmJoyHeader() {
    if (!armjoy_csv_) return;
    armjoy_csv_
      << "recv_time,"
      << "ee_x_cmd,ee_y_cmd,ee_z_cmd,ee_roll_cmd,ee_pitch_cmd,"
      << "waist_cmd,gripper_cmd,pose_cmd,speed_cmd,speed_toggle_cmd,"
      << "gripper_pwm_cmd,torque_cmd\n";
    if (flush_each_row_) armjoy_csv_.flush();
  }

  void writeCombinedHeader() {
    if (!combined_csv_) return;
    combined_csv_ << "recv_time,t_rel,source,joy_header_sec,joy_header_nsec,"
                  << "ee_x_cmd,ee_y_cmd,ee_z_cmd,ee_roll_cmd,ee_pitch_cmd,"
                  << "waist_cmd,gripper_cmd,pose_cmd,speed_cmd,speed_toggle_cmd,"
                  << "gripper_pwm_cmd,torque_cmd";
    for (size_t i=0;i<last_joy_.axes.size();++i)    combined_csv_ << ",axes_"   << i;
    for (size_t j=0;j<last_joy_.buttons.size();++j) combined_csv_ << ",button_" << j;
    combined_csv_ << "\n";
    if (flush_each_row_) combined_csv_.flush();
  }

  void writeJoyRow(double t_recv, double t_rel, const sensor_msgs::Joy& joy) {
    if (!joy_csv_) return;
    joy_csv_ << std::fixed << std::setprecision(6)
             << t_recv << "," << t_rel << ","
             << joy.header.stamp.sec << "," << joy.header.stamp.nsec << ","
             << joy.header.seq;
    for (double v : joy.axes)   joy_csv_ << "," << v;
    for (int b : joy.buttons)   joy_csv_ << "," << b;
    joy_csv_ << "\n";
    if (flush_each_row_) joy_csv_.flush();
  }

  void writeCalRow(double t_recv, double t_rel, const sensor_msgs::Joy& joy) {
    if (!cal_csv_) return;
    auto safeAxis = [&](const char* k)->double {
      int i = cntlr_.at(k);
      return (i >=0 && (size_t)i < joy.axes.size()) ? joy.axes[i] : 0.0;
    };
    cal_csv_ << std::fixed << std::setprecision(6)
             << t_recv << "," << t_rel << ","
             << safeAxis("EE_X")    << ","
             << safeAxis("EE_Z")    << ","
             << safeAxis("EE_ROLL") << ","
             << safeAxis("EE_PITCH") << "\n";
    if (flush_each_row_) cal_csv_.flush();
  }

  void writeArmJoyRow(double t_recv, const interbotix_xs_msgs::ArmJoy& m) {
    if (!armjoy_csv_) return;
    armjoy_csv_ << std::fixed << std::setprecision(6)
                << t_recv << ","
                << static_cast<int>(m.ee_x_cmd) << ","
                << static_cast<int>(m.ee_y_cmd) << ","
                << static_cast<int>(m.ee_z_cmd) << ","
                << static_cast<int>(m.ee_roll_cmd) << ","
                << static_cast<int>(m.ee_pitch_cmd) << ","
                << static_cast<int>(m.waist_cmd) << ","
                << static_cast<int>(m.gripper_cmd) << ","
                << static_cast<int>(m.pose_cmd) << ","
                << static_cast<int>(m.speed_cmd) << ","
                << static_cast<int>(m.speed_toggle_cmd) << ","
                << static_cast<int>(m.gripper_pwm_cmd) << ","
                << static_cast<int>(m.torque_cmd) << "\n";
    if (flush_each_row_) armjoy_csv_.flush();
  }

  void writeCombinedRow(double t_recv, double t_rel, const char* source) {
    if (!combined_csv_) return;
    combined_csv_ << std::fixed << std::setprecision(6)
                  << t_recv << "," << t_rel << "," << source << ",";
    if (have_joy_) combined_csv_ << last_joy_.header.stamp.sec << "," << last_joy_.header.stamp.nsec << ",";
    else           combined_csv_ << "0,0,";

    if (have_armjoy_) {
      const auto& m = last_armjoy_;
      combined_csv_ << static_cast<int>(m.ee_x_cmd) << ","
                    << static_cast<int>(m.ee_y_cmd) << ","
                    << static_cast<int>(m.ee_z_cmd) << ","
                    << static_cast<int>(m.ee_roll_cmd) << ","
                    << static_cast<int>(m.ee_pitch_cmd) << ","
                    << static_cast<int>(m.waist_cmd) << ","
                    << static_cast<int>(m.gripper_cmd) << ","
                    << static_cast<int>(m.pose_cmd) << ","
                    << static_cast<int>(m.speed_cmd) << ","
                    << static_cast<int>(m.speed_toggle_cmd) << ","
                    << static_cast<int>(m.gripper_pwm_cmd) << ","
                    << static_cast<int>(m.torque_cmd);
    } else {
      combined_csv_ << "0,0,0,0,0,0,0,0,0,0,0,0";
    }

    if (have_joy_) {
      for (double v : last_joy_.axes)    combined_csv_ << "," << v;
      for (int b : last_joy_.buttons)    combined_csv_ << "," << b;
    }
    combined_csv_ << "\n";
    if (flush_each_row_) combined_csv_.flush();
  }

private:
  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber joy_sub_, armjoy_sub_, csv_base_sub_;
  ros::Timer timer_;
  std::mutex mtx_;

  // Params / config
  std::string basename_topic_;
  std::string joy_topic_, armjoy_topic_;
  std::string base_dir_;
  std::string controller_type_;
  std::map<std::string,int> cntlr_;
  double on_thr_ = 0.75;
  bool write_combined_ = true;
  bool append_ = false;
  bool flush_each_row_ = false;
  double log_rate_hz_ = 10.0;

  // CSV files and paths
  std::ofstream joy_csv_, armjoy_csv_, combined_csv_, cal_csv_;
  std::string joy_csv_path_, armjoy_csv_path_, combined_csv_path_, cal_csv_path_;
  bool joy_header_written_ = false, armjoy_header_written_ = false, combined_header_written_ = false;

  // State
  sensor_msgs::Joy last_joy_; bool have_joy_ = false;
  interbotix_xs_msgs::ArmJoy last_armjoy_; bool have_armjoy_ = false;

  // Active base (id_task_###) from session; empty means idle
  std::string active_base_;

  // Session origin for t_rel
  ros::Time session_t0_;
  bool have_t0_ = false;

  // (TF objects kept for parity; not used here)
  tf::TransformListener tf_listener_;
  tf::StampedTransform last_tf_;
  bool have_tf_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_dual_logger");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  JoyDualLogger node(nh, pnh);
  ros::spin();
  return 0;
}
