# mavlink_utilities.py
from pymavlink.dialects.v20 import cubepilot 
from pymavlink import mavutil
from time import sleep
import time

def setup_connection(connection_string):
    """Setup MAVLink connection and return the connection object."""
    the_connection = mavutil.mavlink_connection(connection_string)
    the_connection.wait_heartbeat()
    print(f"Heartbeat from system (system {the_connection.target_system} component {the_connection.target_component})")
    return the_connection

def arm_vehicle(connection):
    """Send command to arm the vehicle."""
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )

def disarm_vehicle(connection):
    """Send command to disarm the vehicle."""
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )

def set_position(connection, destination):
    """Send position command."""
    connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, connection.target_system,
                        connection.target_component, 
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, int(0b110111111000), 
                        int(destination[0] * 10 ** 7), int(destination[1] * 10 ** 7), 
                        20, 0, 0, 0, 0, 0, 0, 1.57, 0.5))

def setX_velocity_and_yaw(connection, boot_time, Vx, Yaw):
   """Send velocity and yaw command."""
   connection.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)), # ms since boot
        connection.target_system, connection.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(0b100111100111), lat_int=0, lon_int=0, alt=15, 
        vx=Vx, vy=0, vz=0, # velocities in NED frame [m/s] 
        afx=0, afy=0, afz=0, yaw=Yaw, yaw_rate=0)

def control_rc_channels(connection, chan1, chan2):
    """Control the RC channels over a specified duration."""
    connection.mav.rc_channels_override_send(
            connection.target_system,
            connection.target_component,
            chan2,  # chan1_raw (e.g.left motor)
            chan1,   # chan2_raw 
            1500,   # chan3_raw 
            1500,  # chan4_raw (e.g.right motor)
            2000,      # chan5_raw
            0,      # chan6_raw
            0,      # chan7_raw
            0       # chan8_raw
        )

def getHomeLocation(connection):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0,  # Confirmation
        0, 0, 0, 0, 0, 0, 0  # Parameters are unused
    )
    while True:
        # Get a message
        msg = connection.recv_match(type='HOME_POSITION', blocking=True)
        if msg:
            # Extract latitude and longitude
            latitude = msg.latitude / 1e7  # Convert from 1E7 degrees to degrees ##only for debug
            longitude = msg.longitude / 1e7  # Convert from 1E7 degrees to degrees ##only for debug
            print(f"Home position: Latitude = {latitude}, Longitude = {longitude}")
            break  # Exit the loop after printing home position
        else:
            latitude = 0
            longitude = 0

    return [latitude,longitude]
