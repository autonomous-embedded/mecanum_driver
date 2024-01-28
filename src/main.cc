#include <stdint.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include "mecanum.pb.h"

static constexpr double pi{3.14159265359};

serial::Serial uart;

bool in_range(double val, double left, double right) {
  return ((val > left) && (val < right));
}

void handle_geom_twist(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("Received: linear.x: %f", msg->linear.x);

  /* Determine the correct OMEGA value, because someone designed the low-level library
   * whilst being drunk... (my implementation is also terrible, but whatever) */
  double omega = pi/2;
  if (in_range(msg->linear.y, -0.25, 0.25) || msg->linear.y == -0.25 || msg->linear.y == 0.25) {
    omega = pi/2;
  } 
  else if (in_range(msg->linear.y, -0.75, -0.25) || msg->linear.y == -0.75) {
    omega = pi;
  }
  else if (in_range(msg->linear.y, -1, -0.75) || in_range(msg->linear.y, 0.75, 1) ||
	   msg->linear.y == -1 || msg->linear.y == 1) {
    omega = 3 * pi/2;
  }
  else if (in_range(msg->linear.y, 0.25, 0.75) || msg->linear.y == 0.75) {
    omega = 0;
  }
  /* As I said, terrible.
   * I'm ashamed when looking at it after it's been written.
   * I don't care really, so it's up to someone more motivated to update it */

  ControlRequest ctrl_req;
  /* A little something for the reader to figure out hehe. Scaling with a magic number like
   * a peasant. It's terrible and I know it.  */
  ctrl_req.set_speed_mmps(static_cast<int32_t>(msg->linear.x * 600));
  ctrl_req.set_omega(omega);
  ctrl_req.set_rad(0);

  std::string buffer;
  if (ctrl_req.SerializeToString(&buffer)) {
    uart.write(buffer);
  }
  else {
    ROS_INFO("serialization error");
  }
}

int main(int argc, char** argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  try {
      uart.setPort("/dev/ttyUSB0");
      uart.setBaudrate(19200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      uart.setTimeout(to);
      uart.open();
  }
  catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if (uart.isOpen()) {
      ROS_INFO_STREAM("Serial Port initialized");
  }
  else {
      return -1;
  }

  ros::init(argc, argv, "mecanum_driver");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 5, handle_geom_twist);

  ros::spin();

  google::protobuf::ShutdownProtobufLibrary();
  return 0; // success
}
