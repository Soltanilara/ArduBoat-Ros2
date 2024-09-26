#standard Library imports for ROS2
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

"""
This is a ROS2 Publisher node to publish velocity and heading in body frame to 
topic /VelocityYaw.The user need to input desinred 
velocity in m/sec and heading in degrees
"""
"""
Filename: velocityYawSetter.py
Description: ROS2 Publisher for publishing velocity and heading setpoints.
Author: Dinesh Kumar
Date: 2024-04
"""

class DestinationPublisher(Node): #publisher node class
    
    
    def __init__(self):
        super().__init__('position_publisher')
        self.running = True
        self.SetValuePublisher_ = self.create_publisher(Float32MultiArray, 'VelocityYaw', 10)
        self.latest_Vx = 0
        self.latest_Yaw = 0
        self.listen_for_input()

    def publish_values(self):
        msg = Float32MultiArray()
        msg.data = [self.latest_Vx, self.latest_Yaw]
        self.SetValuePublisher_.publish(msg)

    def listen_for_input(self): #Waits for User function and then publishes value to topic
        while self.running:
            temp_input1 = input("Enter desired velocity and Yaw: ") ##insert checks for values
            #temp_input2 = input("Enter new value for Left Thruster: ")
            str_values = temp_input1.split(',')
            try:
                self.latest_Vx = float(str_values[0])  
                self.latest_Yaw = float(str_values[1]) 
                self.latest_Yaw = math.radians(self.latest_Yaw) 
                self.publish_values()
            except ValueError:
                self.get_logger().info(f'Publishing: Invalid input.Please enter float values seperated by comma')

def main(args=None):
    
    rclpy.init(args=args)
    node = DestinationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:    
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
