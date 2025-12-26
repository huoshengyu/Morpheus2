// tf_pose_logger.cpp
#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <mutex>
#include <filesystem>
#include <cmath>   // std::round

#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// ============================= CSV helper (per-trial rotation) =============================
class CSVLogger {
public:
  explicit CSVLogger(ros::NodeHandle& pnh) {
    // Paths & behavior
    pnh.param<std::string>("base_dir", base_dir_, std::string("/root/catkin_ws/src/morpheus_data/data/"));
    pnh.param<bool>("append", append_, false);
    pnh.param<bool>("flush_each_row", flush_each_row_, false);

    // Save-time clamp & quantization
    pnh.param("save_pos_min_m",        save_pos_min_m_,       -1e9);
    pnh.param("save_pos_max_m",        save_pos_max_m_,        1e9);
    pnh.param("save_pos_resolution_m", save_pos_resolution_m_, 0.0);  // 0 = none
    pnh.param("save_quat_resolution",  save_quat_resolution_,  0.0);  // 0 = none

    // Formatting
    pnh.param("csv_precision", csv_precision_, 20);
  }

  // Open/rotate to new basename ("" = end session)
  void reopenWithBase(const std::string& basename) {
    std::lock_guard<std::mutex> lk(mtx_);
    closeUnlocked();

    active_base_ = basename;
    if (active_base_.empty()) {
      header_written_ = false;
      return; // idle between trials
    }

    // Ensure dir exists
    try { std::filesystem::create_directories(base_dir_); }
    catch (...) { ROS_WARN_STREAM("[TFPoseLogger] Could not ensure directory: " << base_dir_); }

    const std::string base_full = joinPath(base_dir_, active_base_);
    csv_path_ = base_full + "_tf_pose.csv";

    std::ios_base::openmode mode = std::ios::out | (append_ ? std::ios::app : std::ios::trunc);
    ofs_.open(csv_path_.c_str(), mode);
    if (!ofs_) {
      ROS_ERROR_STREAM("[TFPoseLogger] Failed to open CSV: " << csv_path_);
      active_base_.clear();
      return;
    }
    ofs_.setf(std::ios::fixed);
    ofs_.precision(csv_precision_);
    header_written_ = false;

    ROS_INFO_STREAM("[TFPoseLogger] Writing to: " << csv_path_
                    << " (precision=" << csv_precision_
                    << ", clamp=[" << save_pos_min_m_ << "," << save_pos_max_m_
                    << "], pos_res=" << save_pos_resolution_m_
                    << ", quat_res=" << save_quat_resolution_ << ")");
  }

  void endSession() { reopenWithBase(""); }

  bool isActive() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return ofs_.is_open();
  }

  void writeHeaderIfNeeded() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ofs_ || header_written_) return;

    ofs_ << "stamp,t_rel";
    auto add_pose_cols = [&](const std::string& base){
      ofs_ << "," << base << "_x"
           << "," << base << "_y"
           << "," << base << "_z"
           << "," << base << "_qx"
           << "," << base << "_qy"
           << "," << base << "_qz"
           << "," << base << "_qw";
    };
    add_pose_cols("gripper_pos");
    add_pose_cols("wrist_pos");
    add_pose_cols("shoulder_pos");
    add_pose_cols("upper_arm_pos");
    add_pose_cols("upper_forearm_pos");
    add_pose_cols("ee_arm_pos");
    add_pose_cols("lower_forearm_pos");
    ofs_ << "\n";
    if (flush_each_row_) ofs_.flush();

    header_written_ = true;
  }

  // New: write a row including t_rel
  void writeRowFromMsgsWithTRel(
    double t_stamp, double t_rel,
    const geometry_msgs::TransformStamped& gr,
    const geometry_msgs::TransformStamped& wr,
    const geometry_msgs::TransformStamped& sh,
    const geometry_msgs::TransformStamped& ua,
    const geometry_msgs::TransformStamped& uf,
    const geometry_msgs::TransformStamped& ee,
    const geometry_msgs::TransformStamped& lf
  ){
    std::lock_guard<std::mutex> lk(mtx_);
    if (!ofs_) return;

    ofs_ << std::fixed << std::setprecision(csv_precision_) << t_stamp
         << "," << t_rel;

    auto clamp_pos = [&](double v) {
      if (v < save_pos_min_m_) v = save_pos_min_m_;
      if (v > save_pos_max_m_) v = save_pos_max_m_;
      return v;
    };
    auto quantize = [&](double v, double res){
      if (res <= 0.0) return v;
      return std::round(v / res) * res;
    };

    auto pose = [&](const geometry_msgs::TransformStamped& m){
      // positions: clamp + optional quantize
      double x = quantize(clamp_pos(m.transform.translation.x), save_pos_resolution_m_);
      double y = quantize(clamp_pos(m.transform.translation.y), save_pos_resolution_m_);
      double z = quantize(clamp_pos(m.transform.translation.z), save_pos_resolution_m_);

      // quaternions: optional quantize (no clamp)
      double qx = quantize(m.transform.rotation.x, save_quat_resolution_);
      double qy = quantize(m.transform.rotation.y, save_quat_resolution_);
      double qz = quantize(m.transform.rotation.z, save_quat_resolution_);
      double qw = quantize(m.transform.rotation.w, save_quat_resolution_);

      ofs_ << "," << x << "," << y << "," << z
           << "," << qx << "," << qy << "," << qz << "," << qw;
    };

    pose(gr); pose(wr); pose(sh); pose(ua); pose(uf); pose(ee); pose(lf);
    ofs_ << "\n";
    if (flush_each_row_) ofs_.flush();
  }

private:
  static std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/' || a.back() == '\\') return a + b;
    return a + "/" + b;
  }

  void closeUnlocked() {
    if (ofs_.is_open()) ofs_.close();
  }

  // Params / state
  std::string base_dir_;
  bool append_{false};
  bool flush_each_row_{false};

  std::ofstream ofs_;
  std::string csv_path_;
  std::string active_base_;
  bool header_written_{false};
  int csv_precision_{20};
  mutable std::mutex mtx_;

  // Save-time controls
  double save_pos_min_m_{-1e9};
  double save_pos_max_m_{ 1e9};
  double save_pos_resolution_m_{0.0};
  double save_quat_resolution_{0.0};
};

// ============================= TopicPublisher (timer-driven) =============================
class TopicPublisher {
public:
  TopicPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh, CSVLogger& csv)
  : csv_(csv), listener_(ros::Duration(10.0)) // TF cache
  {
    pnh.param<std::string>("world_frame", world_, std::string("vx300s/base_link"));
    pnh.param("log_rate_hz", log_rate_hz_, 10.0);

    children_ = {
      "vx300s/gripper_link",
      "vx300s/wrist_link",
      "vx300s/shoulder_link",
      "vx300s/upper_arm_link",
      "vx300s/upper_forearm_link",
      "vx300s/ee_arm_link",
      "vx300s/lower_forearm_link"
    };

    // Pose publishers (optional visibility)
    gripper_pos_pub         = nh.advertise<geometry_msgs::TransformStamped>("gripper_pos", 10);
    wrist_pos_pub           = nh.advertise<geometry_msgs::TransformStamped>("wrist_pos", 10);
    shoulder_pos_pub        = nh.advertise<geometry_msgs::TransformStamped>("shoulder_pos", 10);
    upper_arm_pos_pub       = nh.advertise<geometry_msgs::TransformStamped>("upper_arm_pos", 10);
    upper_forearm_pos_pub   = nh.advertise<geometry_msgs::TransformStamped>("upper_forearm_pos", 10);
    ee_arm_pos_pub          = nh.advertise<geometry_msgs::TransformStamped>("ee_arm_pos", 10);
    lower_forearm_pos_pub   = nh.advertise<geometry_msgs::TransformStamped>("lower_forearm_pos", 10);

    // Uniform sampling via ROS Timer (ROS time, not wall time)
    const double hz = std::max(1e-6, log_rate_hz_);
    timer_ = nh.createTimer(ros::Duration(1.0 / hz),
                            &TopicPublisher::onTimer, this, /*oneshot=*/false, /*autostart=*/true);

    ROS_INFO_STREAM("[TFPoseLogger] world=" << world_
                    << " | log_rate_hz=" << log_rate_hz_);
  }

  // Called by node wrapper on session start/end to align t0 with joy logger
  void startSessionT0() { session_t0_ = ros::Time::now(); have_t0_ = true; }
  void endSession()     { have_t0_ = false; }

private:
  void onTimer(const ros::TimerEvent&) {
    if (!csv_.isActive() || !have_t0_) return;  // only when a trial is active and we have t0
    publishOnce();
  }

  void publishOnce() {
    // Use a uniform, timer-based stamp for the row
    const ros::Time row_stamp = ros::Time::now();
    const double t_rel = (row_stamp - session_t0_).toSec();

    // Lookup latest-available transforms (Time(0))
    tf::StampedTransform gr, wr, sh, ua, uf, ee, lf;
    auto lookup_pose_latest = [&](const std::string& child, tf::StampedTransform& out)->bool {
      try {
        listener_.waitForTransform(world_, child, ros::Time(0), ros::Duration(0.02));
        listener_.lookupTransform(world_, child, ros::Time(0), out);
        return true;
      } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF latest lookup failed for %s -> %s: %s",
                          world_.c_str(), child.c_str(), ex.what());
        return false;
      }
    };

    if (!lookup_pose_latest("vx300s/gripper_link",       gr)) return;
    if (!lookup_pose_latest("vx300s/wrist_link",         wr)) return;
    if (!lookup_pose_latest("vx300s/shoulder_link",      sh)) return;
    if (!lookup_pose_latest("vx300s/upper_arm_link",     ua)) return;
    if (!lookup_pose_latest("vx300s/upper_forearm_link", uf)) return;
    if (!lookup_pose_latest("vx300s/ee_arm_link",        ee)) return;
    if (!lookup_pose_latest("vx300s/lower_forearm_link", lf)) return;

    // Build messages stamped with the uniform row time
    auto fill_pose = [&](geometry_msgs::TransformStamped& m,
                         const std::string& frame, const std::string& child,
                         const tf::StampedTransform& tfst){
      m.header.stamp = row_stamp;           // uniform, timer-driven
      m.header.frame_id = frame;
      m.child_frame_id = child;
      m.transform.translation.x = tfst.getOrigin().x();
      m.transform.translation.y = tfst.getOrigin().y();
      m.transform.translation.z = tfst.getOrigin().z();
      m.transform.rotation.x = tfst.getRotation().x();
      m.transform.rotation.y = tfst.getRotation().y();
      m.transform.rotation.z = tfst.getRotation().z();
      m.transform.rotation.w = tfst.getRotation().w();
    };

    geometry_msgs::TransformStamped gr_msg, wr_msg, sh_msg, ua_msg, uf_msg, ee_msg, lf_msg;
    fill_pose(gr_msg, world_, "vx300s/gripper_link",       gr);
    fill_pose(wr_msg, world_, "vx300s/wrist_link",         wr);
    fill_pose(sh_msg, world_, "vx300s/shoulder_link",      sh);
    fill_pose(ua_msg, world_, "vx300s/upper_arm_link",     ua);
    fill_pose(uf_msg, world_, "vx300s/upper_forearm_link", uf);
    fill_pose(ee_msg, world_, "vx300s/ee_arm_link",        ee);
    fill_pose(lf_msg, world_, "vx300s/lower_forearm_link", lf);

    // Publish (optional)
    gripper_pos_pub.publish(gr_msg);
    wrist_pos_pub.publish(wr_msg);
    shoulder_pos_pub.publish(sh_msg);
    upper_arm_pos_pub.publish(ua_msg);
    upper_forearm_pos_pub.publish(uf_msg);
    ee_arm_pos_pub.publish(ee_msg);
    lower_forearm_pos_pub.publish(lf_msg);

    // Ensure header then log row
    csv_.writeHeaderIfNeeded();
    csv_.writeRowFromMsgsWithTRel(
      row_stamp.toSec(), t_rel,
      gr_msg, wr_msg, sh_msg, ua_msg, uf_msg, ee_msg, lf_msg
    );
  }

  CSVLogger& csv_;
  tf::TransformListener listener_;

  // Params
  std::string world_;
  double log_rate_hz_{10.0};

  // Timer (ROS time)
  ros::Timer timer_;

  // Pose publishers (optional)
  ros::Publisher gripper_pos_pub, wrist_pos_pub, shoulder_pos_pub, upper_arm_pos_pub;
  ros::Publisher upper_forearm_pos_pub, ee_arm_pos_pub, lower_forearm_pos_pub;

  // Frames
  std::vector<std::string> children_;

  // Session origin
  ros::Time session_t0_;
  bool have_t0_{false};
};

// ============================= Node wrapper (basename rotation) =============================
class TFPoseLoggerNode {
public:
  TFPoseLoggerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), csv_(pnh_), pub_(nh_, pnh_, csv_)
  {
    pnh_.param<std::string>("basename_topic", basename_topic_, std::string("/UCDsession/csv_basename"));
    csv_base_sub_ = nh_.subscribe(basename_topic_, 1, &TFPoseLoggerNode::csvBaseCb, this);
    ROS_INFO_STREAM("[TFPoseLogger] starting. basename_topic=" << basename_topic_);
  }

  void csvBaseCb(const std_msgs::String::ConstPtr& msg) {
    const std::string s = msg->data;
    if (s.empty()) {
      csv_.endSession();
      pub_.endSession(); // stop using old t0
      ROS_INFO("[TFPoseLogger] Session ended closed CSV; waiting for next basename.");
    } else {
      csv_.reopenWithBase(s);
      pub_.startSessionT0(); // align origin with joy logger
      ROS_INFO_STREAM("[TFPoseLogger] Session started base=" << s);
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  CSVLogger csv_;
  TopicPublisher pub_;
  std::string basename_topic_;
  ros::Subscriber csv_base_sub_;
};

// ============================= Main =============================
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pose_logger");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  TFPoseLoggerNode node(nh, pnh);
  ros::spin();
  return 0;
}
