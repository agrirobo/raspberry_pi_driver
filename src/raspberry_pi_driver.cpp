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

  int start_stop_, interrupt_, reduction_ratio_, wheel_radius_, pulse_per_spin_;
  static int cw_ccw_, pulse_;

public:
  RPiDriver(ros::NodeHandle& node_handle)
    : sub_ {node_handle.subscribe<geometry_msgs::Point>("sub_topic", 1, &RPiDriver::callback, this)},
      pub_ {node_handle.advertise<nav_msgs::Odometry>("pub_topic", 1)},
      command_ {},
      odometry_ {}
  {
    if (wiringPiSetupPhys() < 0) ROS_ERROR_STREAM("[ERROR] wiringPiSetupPhys()");

    node_handle.getParam("tolerance", tolerance_);

    node_handle.getParam("cw_ccw", cw_ccw_);
    node_handle.getParam("start_stop", start_stop_);
    node_handle.getParam("interrupt", interrupt_);

    node_handle.getParam("reduction_ratio", reduction_ratio_);
    node_handle.getParam("wheel_radius", wheel_radius_);
    node_handle.getParam("pulse_per_spin", pulse_per_spin_);

    pinMode(cw_ccw_, OUTPUT);
    pinMode(start_stop_, OUTPUT);
    wiringPiISR(interrupt_, INT_EDGE_RISING, &RPiDriver::interrupt);
  }

  void write()
  {
    if ((cw_ccw_ = command_.x - odometry_.pose.pose.position.x) >= 0) digitalWrite(cw_ccw_, HIGH);
    else digitalWrite(cw_ccw_, LOW);

    if (abs(cw_ccw_) > tolerance_) digitalWrite(start_stop_, HIGH);
    else digitalWrite(start_stop_, LOW);
  }

private:
  void callback(const geometry_msgs::PointConstPtr& command)
  {
    command_ = *command;
    odometry_.pose.pose.position.x = pulse_ * wheel_radius_ * pulse_per_spin_ * reduction_ratio_;
  }

  static void interrupt()
  {
    if (cw_ccw_ >= 0) pulse_++;
    else pulse_--;
  }
};

int RPiDriver::cw_ccw_ {0};
int RPiDriver::pulse_ {0};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_driver_node");
  ros::NodeHandle node_handle {};
  ros::Rate rate {ros::Duration(0.1)};

  RPiDriver driver {node_handle};

  while (ros::ok()) {
    driver.write();
    rate.sleep();
  }

  return 0;
}
