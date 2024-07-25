"""
This is a ROS2 Subscriber node to navigate to destination through mavlink into Ardupilot. Destination is read of topic /Destination. 
"""

"""
Filename: destinationSetter.py
Description: ROS2 subscriber for sending waypoint commands.
Author: Dinesh Kumar
Date: 2024-04
"""
#Standard libraries for ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

#Class Import for mavlink commands
from BoatController import mavlink_utilities

class DestinationCommand(Node): #Subscriber Class for reading topic and navigating
    def __init__(self,home,boat):
        super().__init__('destination_command')
    
        # Initialize variables
        self.home_values = home #Receive home
        self.destination_values = [0,0]
        self.boat = boat
        
        # Subscribers
        self.create_subscription(Float32MultiArray, 'Destination', self.position_callback, 10) 

    def position_callback(self, msg):
        print("Seen new values")
        self.destination_values  = msg.data
        print(type(self.destination_values[0]))
        mavlink_utilities.set_position(self.boat,self.destination_values)  # Send waypoint to boat 
        self.get_logger().info(f'{msg.data[0]}, {msg.data[1]}') #debug purposes


def main(args=None):
    rclpy.init(args=args)
    url = "tcp:localhost:5762"
    """
    url changes according to use case. If in simulation use "tcp:localhost:5762" or udp:localhost:14550. If on physical boat use either "/dev/ttyUSBx" or "/dev/ttyACMx" or "/dev/ttyTHSx"
    """ 
    # Setup MAVLink connection and Position Controller
    boat = mavlink_utilities.setup_connection(url)
    home = mavlink_utilities.getHomeLocation(boat)
    positionSetter = DestinationCommand(home,boat)
    rclpy.spin(positionSetter)
    positionSetter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

