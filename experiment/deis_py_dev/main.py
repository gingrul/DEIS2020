#! /bin/python3

# Ioannis Broumas
# ioabro17@student.hh.se
# Christo George
# christogeorge@live.in
# Nov 2020

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose

class MainNode(Node):
        def __init__(self):
            super().__init__('main_node')
            self.publisher_ = self.create_publisher(String, 'WHEEL_SPEEDS', 10)
            self.teleop_subscription = self.create_subscription(String, 'TELEOP', self.teleop_callback, 10 )
            self.teleop_subscription  # prevent unused variable warning
            #get the drone pose 
            self.drone_pose_sub  = self.create_subscription(Pose, '/drone_pose', self.drone_pose_callback, 10)
            self.drone_pose_sub
            #timer_period = 3
            #self.timer = self.create_timer(timer_period, self.teleop_callback)
            #self.i = 0
            self.get_logger().info('Node Main initialized!')
            
        def teleop_callback(self,msg):
            msg.data = msg.data + '\n'
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing wheel speeds "%s"' %msg.data)
        
        def drone_pose_callback(self, msg):
            self.get_logger().info("Got drone pose update")
        

def main(args=None):
        rclpy.init(args=args)
        main_node = MainNode()
        rclpy.spin(main_node)

        #destroy is optional
        main_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
