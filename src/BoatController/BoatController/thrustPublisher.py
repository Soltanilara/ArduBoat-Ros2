import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from PositionContoller import mavlink_utilities

class ThrusterController(Node):
    def __init__(self,boat=None):
        super().__init__('thruster_controller')
        # Initialize variables
        self.thruster_values = [1500,1500]
        #self.left_thruster_value = 1500
        # boat communication setup
        self.boat = boat
        # Subscribers
        self.create_subscription(Int32MultiArray, '/Thrusters', self.thruster_callback, 10) #change to thruster callback
        #self.create_subscription(Int32, '/leftThruster', self.left_thruster_callback, 10)

        # Timer to call function at 100 Hz
        #self.timer = self.create_timer(0.01, self.publish_thruster_values)

    def thruster_callback(self, msg):
        #mavlink_utilities.control_rc_channels(self.boat, msg.data[0],msg.data[1])
        mavlink_utilities.arm_vehicle(self.boat)
        self.thruster_values  = msg.data
        self.get_logger().info(f'Right Thruster: {msg.data[0]}, Left Thruster: {msg.data[1]}') #debug purposes
        #self.right_thruster_value = int(msg.data)
        #self.get_logger().info(f'Updated Right Thruster: {self.right_thruster_value}')

    def left_thruster_callback(self, msg):
        self.left_thruster_value = int(msg.data)
        #self.get_logger().info(f'Updated Left Thruster: {self.left_thruster_value}')

    def publish_thruster_values(self):
        # Function called at 100Hz to print values 
        #self.get_logger().info(f'ALL SAFETY CHECKS ARE DISABLED: PROCEED AT YOUR OWN RISK')
        mavlink_utilities.control_rc_channels(self.boat, self.right_thruster_value,self.left_thruster_value)
        #self.get_logger().info(f'Right Thruster: {self.right_thruster_value}, Left Thruster: {self.left_thruster_value}') #debu purposes


def main(args=None):
    rclpy.init(args=args)
    
    # Create a standalone Node to access parameters
    node = Node('thruster_controller_node')
    
    # Declare and retrieve the 'url' parameter
    #url_param = node.declare_parameter('url', '/dev/ttyACM0')  # Default value as fallback
    #url = url_param.get_parameter_value().string_value
    url = '/dev/ttyACM0'
    #Setup MAVLink connection and Thruster Controller
    boat = mavlink_utilities.setup_connection(url)
    thruster_controller = ThrusterController(boat)
    
    # Executor setup
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(thruster_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        mavlink_utilities.disarm_vehicle(boat)
        thruster_controller.destroy_node()
        node.destroy_node()
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
