// C library headers
// #include <stdio.h>
// #include <string.h>

// // Linux headers
// #include <fcntl.h> // Contains file controls like O_RDWR
// #include <errno.h> // Error integer and strerror() function
// #include <termios.h> // Contains POSIX terminal control definitions
// #include <unistd.h> // write(), read(), close()

// #include <string>
#include <stdint.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <serial/serial.h>

#include "mecanum.pb.h"

// class UartPort {
//   std::string port;
//   int fd;
//   struct termios tty;
//   uint8_t rx_buffer[256];
//   uint8_t tx_buffer[256];

//  public:
//   UartPort(const std::string& uart_dev) : port{uart_dev} {}
//   ~UartPort() { Close(); }
  
//   void Open(int baudrate) {
//     fd = open(port.c_str(), O_RDWR);
//     // Create new termios struct, we call it 'tty' for convention
//     struct termios tty;

//     // Read in existing settings, and handle any error
//     if(tcgetattr(fd, &tty) != 0) {
//         throw "Error from tcgetattr\n";
//     }

//     tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
//     tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
//     tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
//     tty.c_cflag |= CS8; // 8 bits per byte (most common)
//     tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
//     tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

//     tty.c_lflag &= ~ICANON;
//     tty.c_lflag &= ~ECHO; // Disable echo
//     tty.c_lflag &= ~ECHOE; // Disable erasure
//     tty.c_lflag &= ~ECHONL; // Disable new-line echo
//     tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
//     tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
//     tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

//     tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
//     tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
//     // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
//     // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

//     tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
//     tty.c_cc[VMIN] = 0;

//     // Set in/out baud rate to be 9600
//     cfsetispeed(&tty, B9600);
//     cfsetospeed(&tty, B9600);

//     // Save tty settings, also checking for error
//     if (tcsetattr(fd, TCSANOW, &tty) != 0) {
//         throw "Error from tcsetattr\n";
//     }
//   }

//   void Close() {
//     close(fd);
//   }

//   void Write(const char* buffer, int len) {
//     write(fd, buffer, len);
//   }

//   void Read() {
//     int num_bytes = read(fd, &rx_buffer, 256);

//     if (num_bytes == 0) {
//       ROS_INFO("No bytes were read");
//     }

//     if (num_bytes < 0) {
//       ROS_INFO("UART Error");
//       throw "UART Error";
//     }   
//   }
// };


// UartPort uart("/dev/ttyACM0");

serial::Serial uart;

void handle_geom_twist(const geometry_msgs::Twist::ConstPtr& msg) {
  ROS_INFO("Received: linear.x: %f", msg->linear.x);

  ControlRequest ctrl_req;
  ctrl_req.set_speed_mmps(static_cast<int32_t>(msg->linear.x * 1000));
  ctrl_req.set_omega(static_cast<int32_t>(msg->angular.z*1000));
  ctrl_req.set_rad(0);

  ROS_INFO("Writing serial: speed %d", ctrl_req.speed_mmps());

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
