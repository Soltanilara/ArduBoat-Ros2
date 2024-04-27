##Subscibes to either home or /Destination topic and sends mavlink command to run rover/boat

##-35.3613635,149.1620831
##-35.3599604,149.1611374
##ADD ARMING DISARM FUNCTION
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from BoatController import mavlink_utilities

class VelocityYawCommander(Node):
    def __init__(self,boat,boot_time):
        super().__init__('setVelocityYaw_command')
        # Initialize variables
        #self.defaultValues = [0,0]
        self.Vx = 0
        self.Yaw_rads = 0
        self.boot_time = boot_time
        self.boat = boat
        # Subscriber
        self.create_subscription(Float32MultiArray, 'VelocityYaw', self.subscriber_callback, 10) #change to thruster callback
        self.timer = self.create_timer(0.01, self.publish_velocityYaw_values) #100Hz
        #self.timer2 = self.create_timer(1, self.printValue) #1Hz


    def subscriber_callback(self, msg):
        self.Vx = msg.data[0]
        self.Yaw_rads = msg.data[1]
        self.get_logger().info(f'{msg.data[0]}, {msg.data[1]}') #debug purposes
    
    def publish_velocityYaw_values(self):
        # Function called at 100Hz to print values 
        #print(f'{self.Vx} :: {self.Yaw_rads}')
        mavlink_utilities.setX_velocity_and_yaw(self.boat,self.boot_time,self.Vx,self.Yaw_rads)
        #self.get_logger().info(f'{self.VxYaw_values}') #debug purposes

    
    
    def printValue(self):
        local_position_ned = self.boat.recv_match(type='LOCAL_POSITION_NED', blocking=True) #move to utility function
        if local_position_ned:
            vx = local_position_ned.vx  # Velocity in X (m/s) - North
            print(f"Velocity (NED): vx: {vx} m/s)")   
        
        # Specifically wait for ATTITUDE message
        attitude = self.boat.recv_match(type='ATTITUDE', blocking=True) ##move to utility function
        if attitude:
            yaw = attitude.yaw  # Yaw in radians
            print(f"Yaw: {yaw} radians")

#self.get_logger().info(f'Right Thruster: {self.right_thruster_value}, Left Thruster: {self.left_thruster_value}') #debu purposes


def main(args=None):
    rclpy.init(args=args)
    url = "tcp:localhost:5762" 
    #url = "/dev/ttyACM0"
    # Setup MAVLink connection and Thruster Controller
    boat = mavlink_utilities.setup_connection(url)
    #print("StartingNode")
    boot_time = time.time()
    velocityYawSetter = VelocityYawCommander(boat,boot_time)
    rclpy.spin(velocityYawSetter)
    mavlink_utilities.disarm_vehicle(boat)
    velocityYawSetter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
