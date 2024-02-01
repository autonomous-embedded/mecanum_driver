#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdint.h>

#include "mecanum.pb.h"

static constexpr double PI_VALUE{3.14159265359};

serial::Serial gUartHandle;

const bool InRange(double val, double left, double right) {
  return ((val > left) && (val < right));
}

void HandleGeomTwist(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_DEBUG("Received: linear.x: %f", msg->linear.x);

  /* Determine the correct OMEGA value, because someone designed the low-level
   * library whilst being drunk... (my implementation is also terrible, but
   * whatever) */
  double angle_of_velocity = PI_VALUE / 2;
  if (InRange(msg->linear.y, -0.25, 0.25) || msg->linear.y == -0.25 ||
      msg->linear.y == 0.25) {
    angle_of_velocity = PI_VALUE / 2;
  } else if (InRange(msg->linear.y, -0.75, -0.25) || msg->linear.y == -0.75) {
    angle_of_velocity = PI_VALUE;
  } else if (InRange(msg->linear.y, -1, -0.75) ||
             InRange(msg->linear.y, 0.75, 1) || msg->linear.y == -1 ||
             msg->linear.y == 1) {
    angle_of_velocity = 3 * PI_VALUE / 2;
  } else if (InRange(msg->linear.y, 0.25, 0.75) || msg->linear.y == 0.75) {
    angle_of_velocity = 0;
  }

  /* As I said, terrible.
   * I'm ashamed when looking at it after it's been written.
   * I don't care really, so it's up to someone more motivated to update it */
  ControlRequest ctrl_req;

  /* A little something for the reader to figure out hehe. Scaling with a magic
   * number like a peasant. It's terrible and I know it.  */
  ctrl_req.set_speed_mmps(static_cast<int32_t>(msg->linear.x * 600));
  ctrl_req.set_omega(0);
  ctrl_req.set_rad(angle_of_velocity);

  if (msg->angular.z != 0.0) {
    ctrl_req.set_speed_mmps(0);
    ctrl_req.set_omega(msg->angular.z);
    ctrl_req.set_rad(0.0);
  }

  std::string buffer;
  const bool written = ctrl_req.SerializeToString(&buffer);
  ROS_DEBUG("Buffer: %s", buffer.c_str());
  if (written) {
    gUartHandle.write(buffer);
  } else {
    ROS_INFO("serialization error");
  }
}

int main(int argc, char** argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  try {
    gUartHandle.setPort("/dev/ttyUSB0");
    gUartHandle.setBaudrate(19200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    gUartHandle.setTimeout(to);
    gUartHandle.open();
  } catch (serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if (gUartHandle.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }

  ros::init(argc, argv, "mecanum_driver");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 5, HandleGeomTwist);

  ros::spin();

  google::protobuf::ShutdownProtobufLibrary();
  return 0;  // success
}
