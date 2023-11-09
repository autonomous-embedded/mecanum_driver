#!/usr/bin/python3
import rospy

from geometry_msgs.msg import Twist

import serial

class UART:
    def __init__(self,port,baudrate,timeout) -> None:
        self.UART = serial.Serial(port,baudrate,timeout=timeout)

    # v - speed vector magnitude
    # rho - speed vector angle (90 deg is forwards, 0 deg is to the right)
    # omega - angular velocity
    # the car will either use (v, rho) or (omega), it can only rotate or move linearly
    def send_message(self, v, rho, omega):
        rho = round(rho, 5)
        omega = round(omega, 5)
        checksum = v + rho + omega
        message = (str(v) + ',' + str(rho) + ',' + str(omega) + ',' + str(checksum) + '\n').encode('ascii')
        self.UART.write(message)
    
# data buffer
last_twist_msg = Twist()

# communication = UART('/dev/', 115200, 1) # TODO
# communication.send_message(0,0,0)

def twist_cb(msg):
    rospy.loginfo(f"Received 'Twist' msg:\n{msg}")
    last_twist_msg = msg
    # communication.send_message(0,0,0)

if __name__ == "__main__":
    rospy.init_node('mecanum_driver')
    control_sub = rospy.Subscriber('/cmd_vel', Twist, twist_cb)
    rospy.spin()
    