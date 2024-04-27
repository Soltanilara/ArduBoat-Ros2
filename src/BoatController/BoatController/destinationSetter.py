##Subscibes to either home or /Destination topic and sends mavlink command to run rover/boat

##-35.3613635,149.1620831
##-35.3599604,149.1611374

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from BoatController import mavlink_utilities

class DestinationCommand(Node):
    def __init__(self,home,boat):
        super().__init__('destination_command')
        # Initialize variables
        self.home_values = home
        self.destination_values = [0,0]
        self.boat = boat
        # Subscribers
        self.create_subscription(Float32MultiArray, 'Destination', self.position_callback, 10) #change to thruster callback

    def position_callback(self, msg):
        #mavlink_utilities.control_rc_channels(self.boat, msg.data[0],msg.data[1])
        print("Seen new values")
        self.destination_values  = msg.data
        print(type(self.destination_values[0]))
        mavlink_utilities.set_position(self.boat,self.destination_values)     
        self.get_logger().info(f'{msg.data[0]}, {msg.data[1]}') #debug purposes

    #def publish_thruster_values(self):
        # Function called at 100Hz to print values 
        #self.get_logger().info(f'ALL SAFETY CHECKS ARE DISABLED: PROCEED AT YOUR OWN RISK')
        #mavlink_utilities.control_rc_channels(self.boat, self.right_thruster_value,self.left_thruster_value)
        #self.get_logger().info(f'Right Thruster: {self.right_thruster_value}, Left Thruster: {self.left_thruster_value}') #debu purposes


def main(args=None):
    rclpy.init(args=args)
    #url = "udp:localhost:14550" 
    url = "tcp:localhost:5762"
    # Setup MAVLink connection and Thruster Controller
    boat = mavlink_utilities.setup_connection(url)
    #print(type(boat_connection))
    home = mavlink_utilities.getHomeLocation(boat)
    #print(type(home))
    #home = 0
    print("StartingNode")
    positionSetter = DestinationCommand(home,boat)
    rclpy.spin(positionSetter)
    positionSetter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





# def main(args=None):
#     #setting up mavlink communications
#     url = "/dev/ttyACM2"
#     boat = mavlink_utilities.setup_connection(url)
#     rclpy.init(args=args)
#     thruster_controller = ThrusterController(boat)
#     executor = SingleThreadedExecutor()
#     executor.add_node(thruster_controller)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         thruster_controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
