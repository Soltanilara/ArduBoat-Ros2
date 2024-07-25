"""
This is a ROS2 Publisher node to publish to topic /Destination. The user inputs a lattitude and longitude value for each desired waypoint.
"""

"""
Filename: positionReader.py
Description: ROS2 publisher for sending waypoint commands.
Author: Dinesh Kumar
Date: 2024-04
"""
#Standard imports for ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DestinationPublisher(Node): #Publisher Class for reading user input and publishing to topic
    def __init__(self):
        super().__init__('position_publisher')
        self.PositionPublisher_ = self.create_publisher(Float32MultiArray, 'Destination', 10)
        self.listen_for_input()

    def publish_values(self):
        msg = Float32MultiArray()
        msg.data = [self.latest_latitude, self.latest_longitude]
        self.PositionPublisher_.publish(msg)


    def listen_for_input(self): #This function waits for user input and then publishes the inputs to the topic.
        while True:
            temp_input1 = input("Enter desired latitude and longitude: ")
            #temp_input2 = input("Enter new value for Left Thruster: ")
            str_values = temp_input1.split(',')
            try:
                self.latest_latitude = float(str_values[0])  
                self.latest_longitude = float(str_values[1])
                self.publish_values()

            except ValueError:
                self.get_logger().info(f'Publishing: Invalid input.Please enter float values seperated by comma')


def main(args=None):
    rclpy.init(args=args)
    node = DestinationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
