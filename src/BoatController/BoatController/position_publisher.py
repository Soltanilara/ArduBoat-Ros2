import rclpy
import sys
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
"""
This is a ROS2 Publisher node to publish to topic /Destination. 
The user inputs a latitude and longitude value for each desired waypoint.
"""
"""
Filename: positionReader.py
Description: ROS2 publisher for sending waypoint commands.
Author: Dinesh Kumar
Date: 2024-04
License: 
"""

#Publisher Class for reading user input and publishing to topic
class DestinationPublisher(Node): 
    def __init__(self):
        super().__init__('position_publisher')
        self.PositionPublisher_ = self.create_publisher(Float32MultiArray, 'Destination', 10)
        self.running = True
        self.listen_for_input()
        
    # Publishes value to Destination
    def publish_values(self):
        
        msg = Float32MultiArray()
        msg.data = [self.latest_latitude, self.latest_longitude]
        self.PositionPublisher_.publish(msg)

    # This function waits for user input and then publishes the inputs to the topic.
    def listen_for_input(self): 
        while self.running:
            try:
                temp_input1 = input("Enter desired latitude and longitude: ")
                str_values = temp_input1.split(',')
                self.latest_latitude = float(str_values[0])  
                self.latest_longitude = float(str_values[1])
                self.publish_values()
            except ValueError:
                self.get_logger().info('Invalid input. Please enter float values separated by a comma')
            except KeyboardInterrupt:
                self.signal_handler(None, None)

def main(args=None):
    rclpy.init(args=args)
    node = DestinationPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
