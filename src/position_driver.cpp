#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <wiringPi.h>

template <typename Command>
class RPiDriver {
  ros::Subscriber sub_;
  ros::Publisher pub_;

  Command command_;
  nav_msgs::Odometry odometry_;

  double tolerance_;
  static constexpr double default_tolerance_ {0.1};

  size_t cw_ccw_, start_stop_, interrupt_;
  static constexpr size_t default_cw_ccw_, {1}; // TODO
  static constexpr size_t default_start_stop_ {2}; // TODO
  static constexpr size_t default_interrupt_ {3}; // TODO

  long pulse_;

public:
  RPiDriver(ros::NodeHandle& node_handle)
    : sub_ {node_handle.subscribe<Command>("command", &RPiDriver::callback)},
      pub_ {node_handle.advertise<nav_msgs::Odometry>("odometry", 1)},
      command_ {},
      odometry_ {},
      pulse_ {}
  {
    // wirinPiSetuphogehoge // TODO

    node_handle.param("tolerance", tolerance_, default_tolerance_);
    node_handle.param("cw_ccw", cw_ccw_, default_cw_ccw_);
    node_handle.param("start_stop", start_stop_, default_start_stop_);
    node_handle.param("interrupt", interrupt_, default_interrupt_);

    wiringPiISR(interrupt_, INT_EDGE_RISING, &RPiDriver::interrupt);
  }

  void write()
  {
    if ((cw_ccw_ = command_ - odometry_.pose.pose.position.x) >= 0) digitalWrite(cw_ccw_, HIGH);
    else digitalWrite(cw_ccw_, LOW);

    if (abs(cw_ccw_) > tolerance_) digitalWrite(start_stop_, HIGH);
    else digitalWrite(start_stop_, LOW);
  }

private:
  void callback(const typename Command::ConstPtr& command) { command_ = *command; }

  void interrupt()
  {
    if (cw_ccw_ >= 0) pulse_++;
    else pulse--;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "position_driver_node");
  ros::NodeHandle node_handle {};
  ros::Rate rate {ros::Duration(1.0)};

  RPiDriver<geometry_msgs::Point> driver {node_handle};

  while (ros::ok()) {
    driver.write();
    rate.sleep();
  }

  return 0;
}
