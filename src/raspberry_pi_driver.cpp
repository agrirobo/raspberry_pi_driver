#include <atomic>
#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <wiringPi.h>

class RPiDriver {
  ros::Subscriber sub_;
  ros::Publisher pub_;

  geometry_msgs::Point command_;
  nav_msgs::Odometry odometry_;

  double tolerance_;
  static constexpr double pi_ {3.1415};

  int start_stop_, interrupt_;
  double reduction_ratio_, wheel_radius_, pulse_per_spin_;
  static int cw_ccw_;
  static std::atomic_long pulse_;
  static bool is_cw_;

public:
  RPiDriver(ros::NodeHandle& node_handle)
    : sub_ {node_handle.subscribe<geometry_msgs::Point>("sub_topic", 1, &RPiDriver::callback, this)},
      pub_ {node_handle.advertise<nav_msgs::Odometry>("pub_topic", 1)},
      command_ {},
      odometry_ {}
  {
    if (wiringPiSetupPhys() < 0) ROS_ERROR_STREAM("wiringPiSetupPhys()");

    ros::NodeHandle pnh {"~"};

    assert(pnh.getParam("tolerance", tolerance_));
    assert(pnh.getParam("cw_ccw", cw_ccw_));
    assert(pnh.getParam("start_stop", start_stop_));
    assert(pnh.getParam("interrupt", interrupt_));
    assert(pnh.getParam("reduction_ratio", reduction_ratio_));
    assert(pnh.getParam("wheel_radius", wheel_radius_));
    assert(pnh.getParam("pulse_per_spin", pulse_per_spin_));

    ROS_INFO_STREAM("tolerance: " << tolerance_);
    ROS_INFO_STREAM("cw_ccw: " << cw_ccw_);
    ROS_INFO_STREAM("start_stop: " << start_stop_);
    ROS_INFO_STREAM("intertupt:" << reduction_ratio_);
    ROS_INFO_STREAM("wheel_radius:" << wheel_radius_);
    ROS_INFO_STREAM("pulse_per_spin: " << pulse_per_spin_);
    ROS_INFO_STREAM("reduction_ratio: " << reduction_ratio_);

    pinMode(cw_ccw_, OUTPUT);
    pinMode(start_stop_, OUTPUT);
    wiringPiISR(interrupt_, INT_EDGE_RISING, &RPiDriver::interrupt);
  }

  void write()
  {
    double distance {command_.x - odometry_.pose.pose.position.x};

    if (is_cw_ = (distance > 0)) digitalWrite(cw_ccw_, HIGH);
    else digitalWrite(cw_ccw_, LOW);

    if (RPiDriver::abs(distance) > tolerance_) digitalWrite(start_stop_, HIGH);
    else digitalWrite(start_stop_, LOW);

    odometry_.pose.pose.position.x = pulse_ * wheel_radius_ / pulse_per_spin_ / reduction_ratio_;
    pub_.publish(odometry_);
  }

private:
  void callback(const geometry_msgs::PointConstPtr& command)
  {
    command_ = *command;
    ROS_INFO_STREAM("command: " << command_);
  }

  static void interrupt()
  {
    if (is_cw_) pulse_++;
    else pulse_--;
    // ROS_INFO_STREAM(pulse_);
  }

  template <typename T>
  static T abs(const T& value)
  {
    return value < 0 ? -value : value;
  }
};

int RPiDriver::cw_ccw_ {0};
long RPiDriver::pulse_ {0};
bool RPiDriver::is_cw_ {0};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_driver_node");
  ros::NodeHandle node_handle {};
  ros::Rate rate {ros::Duration(0.1)};

  RPiDriver driver {node_handle};

  while (ros::ok()) {
    driver.write();
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
