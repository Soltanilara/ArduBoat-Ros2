import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from PositionContoller import mavlink_utilities

class GPSSubscriber(Node):
    def __init__(self,connection):
        super().__init__('gps_subscriber')
        self.connection = connection
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps/destination',
            self.gps_callback,
            10
        )
        #self.subscription  # prevent unused variable warning

    def gps_callback(self, msg:NavSatFix):
        latitude = msg.latitude
        longitude = msg.longitude

        mavlink_utilities.set_position(self.connection,latitude,longitude)
        self.get_logger().info(f'Received GPS Fix - Latitude: {latitude}, Longitude: {longitude}')

def main(args=None):
    the_connection = mavlink_utilities.setup_connection('udpin:localhost:14550')
    the_connection.wait_heartbeat()
    mavlink_utilities.arm_vehicle(the_connection)    
    rclpy.init(args=args)
    node = GPSSubscriber(the_connection)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
