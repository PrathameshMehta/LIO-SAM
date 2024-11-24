#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math


class JoySubscriber(Node):  # Keeping the class name but adapting it for /cmd_vel
    def __init__(self):
        super().__init__('joy_subscriber')
        self.target_x = 0.0
        self.target_w = 0.0

        # Subscribe to /cmd_vel instead of /joy
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Listening to /cmd_vel from teleop_twist_keyboard
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.005, self.timer_callback)

    def listener_callback(self, msg):
        # Directly use Twist message data
        self.target_x = msg.linear.x
        self.target_w = msg.angular.z

    def timer_callback(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.target_x
        vel_msg.angular.z = self.target_w
        self.publisher_.publish(vel_msg)


class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.pitch = 0
        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.listener_callback,
            10
        )

    def listener_callback(self, data):
        q0 = data.orientation.x
        q1 = data.orientation.y
        q2 = data.orientation.z
        q3 = data.orientation.w

        self.pitch = math.asin(2 * (q0 * q2 - q1 * q3))


if __name__ == '__main__':
    rclpy.init(args=None)

    joy_subscriber = JoySubscriber()  # Adapted for /cmd_vel
    imu_subscriber = IMUSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joy_subscriber)
    executor.add_node(imu_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

