#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>                 // joystick raw
#include <interbotix_xs_msgs/ArmJoy.h>       // joystick processed (Interbotix)

#include <filesystem>
#include <string>

// ====================== TF → topics publisher ======================
class TopicPublisher {
public:
  TopicPublisher(ros::NodeHandle& nh)
  {
    // pose topics
    gripper_pos_pub        = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/gripper_pos", 10);
    wrist_pos_pub          = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/wrist_pos", 10);
    shoulder_pos_pub       = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/shoulder_pos", 10);
    upper_arm_pos_pub      = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/upper_arm_pos", 10);
    upper_forearm_pos_pub  = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/upper_forearm_pos", 10);
    ee_arm_pos_pub         = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/ee_arm_pos", 10);
    lower_forearm_pos_pub  = nh.advertise<geometry_msgs::TransformStamped>("/vx300s/lower_forearm_pos", 10);

    // twist topics
    gripper_twist_pub       = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/gripper_vel", 10);
    wrist_twist_pub         = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/wrist_vel", 10);
    shoulder_twist_pub      = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/shoulder_vel", 10);
    upper_arm_twist_pub     = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/upper_arm_vel", 10);
    upper_forearm_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/upper_forearm_vel", 10);
    ee_arm_twist_pub        = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/ee_arm_vel", 10);
    lower_forearm_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/vx300s/lower_forearm_vel", 10);
  }

  // called every loop; bag pointer can be null
  void publishCartesian(rosbag::Bag* bag)
  {
    static tf::TransformListener listener;

    tf::StampedTransform gripper_tf, wrist_tf, shoulder_tf, upper_arm_tf,
                         upper_forearm_tf, ee_arm_tf, lower_forearm_tf;

    geometry_msgs::TransformStamped gripper_msg, wrist_msg, shoulder_msg,
                                    upper_arm_msg, upper_forearm_msg,
                                    ee_arm_msg, lower_forearm_msg;

    geometry_msgs::Twist gripper_twist, wrist_twist, shoulder_twist,
                         upper_arm_twist, upper_forearm_twist, ee_arm_twist,
                         lower_forearm_twist;

    geometry_msgs::TwistStamped gripper_twist_msg, wrist_twist_msg,
                                shoulder_twist_msg, upper_arm_twist_msg,
                                upper_forearm_twist_msg, ee_arm_twist_msg,
                                lower_forearm_twist_msg;

    try {
      // TFs
      listener.lookupTransform("vx300s/base_link", "vx300s/gripper_link",       ros::Time(0), gripper_tf);
      listener.lookupTransform("vx300s/base_link", "vx300s/wrist_link",         ros::Time(0), wrist_tf);
      listener.lookupTransform("vx300s/base_link", "vx300s/shoulder_link",      ros::Time(0), shoulder_tf);
      listener.lookupTransform("vx300s/base_link", "vx300s/upper_arm_link",     ros::Time(0), upper_arm_tf);
      listener.lookupTransform("vx300s/base_link", "vx300s/upper_forearm_link", ros::Time(0), upper_forearm_tf);
      listener.lookupTransform("vx300s/base_link", "vx300s/ee_arm_link",        ros::Time(0), ee_arm_tf);
      listener.lookupTransform("vx300s/base_link", "vx300s/lower_forearm_link", ros::Time(0), lower_forearm_tf);

      // twists (velocities)
      listener.lookupTwist("vx300s/gripper_link",       "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), gripper_twist);

      listener.lookupTwist("vx300s/wrist_link",         "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), wrist_twist);

      listener.lookupTwist("vx300s/shoulder_link",      "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), shoulder_twist);

      listener.lookupTwist("vx300s/upper_arm_link",     "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), upper_arm_twist);

      listener.lookupTwist("vx300s/upper_forearm_link", "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), upper_forearm_twist);

      listener.lookupTwist("vx300s/ee_arm_link",        "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), ee_arm_twist);

      listener.lookupTwist("vx300s/lower_forearm_link", "vx300s/base_link", "vx300s/base_link",
                           tf::Point(0,0,0), "vx300s/base_link",
                           ros::Time(0), ros::Duration(0.1), lower_forearm_twist);
    }
    catch (tf::TransformException &ex) {
      ROS_WARN_THROTTLE(2.0, "%s", ex.what());
      return;
    }

    ros::Time now = ros::Time::now();

    // helper to fill transform messages
    auto fill_tf_msg = [&](geometry_msgs::TransformStamped& m,
                           const tf::StampedTransform& in,
                           const std::string& child){
      m.header.stamp = now;
      m.header.frame_id = "vx300s/base_link";
      m.child_frame_id  = child;
      m.transform.translation.x = in.getOrigin().x();
      m.transform.translation.y = in.getOrigin().y();
      m.transform.translation.z = in.getOrigin().z();
      m.transform.rotation.x = in.getRotation().x();
      m.transform.rotation.y = in.getRotation().y();
      m.transform.rotation.z = in.getRotation().z();
      m.transform.rotation.w = in.getRotation().w();
    };

    fill_tf_msg(gripper_msg,       gripper_tf,       "gripper_link");
    fill_tf_msg(wrist_msg,         wrist_tf,         "wrist_link");
    fill_tf_msg(shoulder_msg,      shoulder_tf,      "shoulder_link");
    fill_tf_msg(upper_arm_msg,     upper_arm_tf,     "upper_arm_link");
    fill_tf_msg(upper_forearm_msg, upper_forearm_tf, "upper_forearm_link");
    fill_tf_msg(ee_arm_msg,        ee_arm_tf,        "ee_arm_link");
    fill_tf_msg(lower_forearm_msg, lower_forearm_tf, "lower_forearm_link");

    // publish transforms
    gripper_pos_pub.publish(gripper_msg);
    wrist_pos_pub.publish(wrist_msg);
    shoulder_pos_pub.publish(shoulder_msg);
    upper_arm_pos_pub.publish(upper_arm_msg);
    upper_forearm_pos_pub.publish(upper_forearm_msg);
    ee_arm_pos_pub.publish(ee_arm_msg);
    lower_forearm_pos_pub.publish(lower_forearm_msg);

    // helper to fill twist messages
    auto fill_twist_msg = [&](geometry_msgs::TwistStamped& out,
                              const geometry_msgs::Twist& in,
                              const std::string& frame){
      out.header.stamp = now;
      out.header.frame_id = frame;
      out.twist = in;
    };

    fill_twist_msg(gripper_twist_msg,       gripper_twist,       "gripper_link");
    fill_twist_msg(wrist_twist_msg,         wrist_twist,         "wrist_link");
    fill_twist_msg(shoulder_twist_msg,      shoulder_twist,      "shoulder_link");
    fill_twist_msg(upper_arm_twist_msg,     upper_arm_twist,     "upper_arm_link");
    fill_twist_msg(upper_forearm_twist_msg, upper_forearm_twist, "upper_forearm_link");
    fill_twist_msg(ee_arm_twist_msg,        ee_arm_twist,        "ee_arm_link");
    fill_twist_msg(lower_forearm_twist_msg, lower_forearm_twist, "lower_forearm_link");

    // publish twists
    gripper_twist_pub.publish(gripper_twist_msg);
    wrist_twist_pub.publish(wrist_twist_msg);
    shoulder_twist_pub.publish(shoulder_twist_msg);
    upper_arm_twist_pub.publish(upper_arm_twist_msg);
    upper_forearm_twist_pub.publish(upper_forearm_twist_msg);
    ee_arm_twist_pub.publish(ee_arm_twist_msg);
    lower_forearm_twist_pub.publish(lower_forearm_twist_msg);

    // write to bag if present
    if (bag) {
      bag->write("/vx300s/gripper_pos",       now, gripper_msg);
      bag->write("/vx300s/wrist_pos",         now, wrist_msg);
      bag->write("/vx300s/shoulder_pos",      now, shoulder_msg);
      bag->write("/vx300s/upper_arm_pos",     now, upper_arm_msg);
      bag->write("/vx300s/upper_forearm_pos", now, upper_forearm_msg);
      bag->write("/vx300s/ee_arm_pos",        now, ee_arm_msg);
      bag->write("/vx300s/lower_forearm_pos", now, lower_forearm_msg);

      bag->write("/vx300s/gripper_vel",       now, gripper_twist_msg);
      bag->write("/vx300s/wrist_vel",         now, wrist_twist_msg);
      bag->write("/vx300s/shoulder_vel",      now, shoulder_twist_msg);
      bag->write("/vx300s/upper_arm_vel",     now, upper_arm_twist_msg);
      bag->write("/vx300s/upper_forearm_vel", now, upper_forearm_twist_msg);
      bag->write("/vx300s/ee_arm_vel",        now, ee_arm_twist_msg);
      bag->write("/vx300s/lower_forearm_vel", now, lower_forearm_twist_msg);
    }
  }

private:
  // pose pubs
  ros::Publisher gripper_pos_pub, wrist_pos_pub, shoulder_pos_pub,
                 upper_arm_pos_pub, upper_forearm_pos_pub,
                 ee_arm_pos_pub, lower_forearm_pos_pub;
  // twist pubs
  ros::Publisher gripper_twist_pub, wrist_twist_pub, shoulder_twist_pub,
                 upper_arm_twist_pub, upper_forearm_twist_pub,
                 ee_arm_twist_pub, lower_forearm_twist_pub;
};

// ====================== Node wrapper with runtime bag name ======================
class TFBagLogger {
public:
  TFBagLogger(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), pub_(nh)
  {
    pnh_.param<std::string>("basename_topic", basename_topic_, std::string("/UCDsession/csv_basename"));
    pnh_.param<std::string>("base_dir",       base_dir_,       std::string("/tmp/"));
    pnh_.param<std::string>("suffix",         suffix_,         std::string("_tf.bag"));

    // start/stop messages for bag
    basename_sub_ = nh_.subscribe(basename_topic_, 1, &TFBagLogger::basenameCb, this);

    // joystick topics to record
    joy_raw_sub_       = nh_.subscribe("/vx300s/joy", 50, &TFBagLogger::joyRawCb, this);
    joy_processed_sub_ = nh_.subscribe("/vx300s/commands/joy_processed", 50, &TFBagLogger::joyProcessedCb, this);

    ROS_INFO_STREAM("[TFBagLogger] listening on " << basename_topic_
                    << " base_dir=" << base_dir_ << " suffix=" << suffix_);
  }

  void spin()
  {
    // 10 Hz → everything recorded at same rate
    ros::Rate r(10);
    while (ros::ok()) {
      pub_.publishCartesian(bag_.isOpen() ? &bag_ : nullptr);
      ros::spinOnce();
      r.sleep();
    }
    if (bag_.isOpen())
      bag_.close();
  }

private:
  // utility for path joining
  static std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (a.back() == '/' || a.back() == '\\') return a + b;
    return a + "/" + b;
  }

  void basenameCb(const std_msgs::String::ConstPtr& msg)
  {
    const std::string base = msg->data;
    if (base.empty()) {
      // end session
      if (bag_.isOpen()) {
        bag_.close();
        ROS_INFO("[TFBagLogger] session ended, bag closed.");
      }
      return;
    }

    // ensure directory
    try {
      std::filesystem::create_directories(base_dir_);
    } catch (...) {
      ROS_WARN_STREAM("[TFBagLogger] could not create directory " << base_dir_);
    }

    std::string fullpath = joinPath(base_dir_, base) + suffix_;

    if (bag_.isOpen())
      bag_.close();

    try {
      bag_.open(fullpath, rosbag::bagmode::Write);
      ROS_INFO_STREAM("[TFBagLogger] opened bag: " << fullpath);
    } catch (...) {
      ROS_ERROR_STREAM("[TFBagLogger] failed to open bag: " << fullpath);
    }
  }

  // joystick raw
  void joyRawCb(const sensor_msgs::Joy::ConstPtr& msg)
  {
    if (!bag_.isOpen())
      return;
    ros::Time t = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    bag_.write("/vx300s/joy", t, *msg);
  }

  // joystick processed
  void joyProcessedCb(const interbotix_xs_msgs::ArmJoy::ConstPtr& msg)
  {
    if (!bag_.isOpen())
      return;
    ros::Time t = ros::Time::now();
    bag_.write("/vx300s/commands/joy_processed", t, *msg);
  }

  ros::NodeHandle nh_, pnh_;
  TopicPublisher pub_;

  std::string basename_topic_;
  std::string base_dir_;
  std::string suffix_;

  ros::Subscriber basename_sub_;
  ros::Subscriber joy_raw_sub_;
  ros::Subscriber joy_processed_sub_;

  rosbag::Bag bag_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_bag_logger");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  TFBagLogger node(nh, pnh);
  node.spin();
  return 0;
}
