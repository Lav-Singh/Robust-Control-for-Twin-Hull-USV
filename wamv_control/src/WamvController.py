#!/usr/bin/env python3
import rospy
import numpy as np
import control as ctrl
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Point, Twist, Vector3Stamped
from std_msgs.msg import Float32
import math
import pymap3d
from Tools.controller import PIDController,KalmanFilter
import signal
import sys
import csv

'''
// Project Title    : Robust Control for twin-hull USV for lagoon surveillance
// Purpose          : Research and Development of a control loop in a simulated environment
// Language         : Python
// Author           : Ramessur Lav Singh
// Date             : 8 September 2024

// Universite des Mascareigns (UdM)
// Master Artifical Intelligence and Robotics
'''


# Initialize lists to store the logged data
time_log = []
lin_vel_log = []
ang_vel_log = []

# Initialize lists to store the actual and expected paths
actual_positions = []  # List to store (x, y) positions of the USV
expected_positions = []  # List to store waypoint positions

# Initialize the PID controllers for linear and angular velocities
linear_pid_nav = PIDController(kP=1.0, kI=0.01, kD=0.05, kS=50)  # Navigation PID
angular_pid_nav = PIDController(kP=21.0, kI=0.27, kD=2.4, kS=50)  # Navigation PID
linear_pid_station = PIDController(kP=1.9, kI=0.12, kD=1.8, kS=50)  # Station-Keeping PID
angular_pid_station = PIDController(kP=1.7, kI=0.11, kD=1.3, kS=50)  # Station-Keeping PID

# Global variables for station-keeping
station_keeping_start_time = None
station_keeping_duration = 10 # Duration in seconds

# Control parameters
v_const = 5.3  # Constant for velocity scaling
v_limit = 1.8  # Max linear velocity
distance_threshold = 5  # Threshold to switch to station-keeping mode

# LOS GUIDANCE PARAMETERS
LOOKAHEAD_DISTANCE = 4.5  # tune it according to performance

# WAYPOINTS TO VISIT
# used for developmennt of Control system, replace with topic later
coordinates = [
    (-33.722414, 150.673977),
    (-33.72204211919601, 150.67408366868685),
    (-33.72217824809369, 150.67445846299378),
    (-33.72258309593731, 150.67409454091023),
]
# Global index for waypoints
current_waypoint_index = 0
final_waypoint_reached = False

# Control mode(True = PID, False = HInf)
CONTROL_PID = True
# ROS parameters for topics
LEFT_THRUSTER_TOPIC = "/wamv/thrusters/left_thrust_cmd"
RIGHT_THRUSTER_TOPIC = "/wamv/thrusters/right_thrust_cmd"
IMU_TOPIC = "/wamv/sensors/imu/imu/data"
GPS_TOPIC = "/wamv/sensors/gps/gps/fix"
WAYPOINT_TOPIC = "/wamv/waypoint"
FIX_VELOCITY = "/wamv/fix_velocity"

# Parameters from WAM-V dynamics 
mass = 180.0  # kg
inertia_z = 446.0  # kg*m^2 (yaw axis inertia)

# Linear drag coefficients (from URDF)
xU = 51.3  # surge linear drag
yV = 40.0  # sway linear drag
nR = 400.0  # yaw linear drag

# State-space matrices based on 3DOF WAM-V dynamics (surge, sway, yaw)
A = np.array([[0.3986 , 0, 0],
              [0, -0.0659, 2.6549],
              [0, 0.2655, -2.0048]])

B = np.array([[0.0078, 0],  # Surge
              [0, 0.0066],  # Sway
              [0, -0.0050]])  # Yaw

C = np.eye(3)  # Identity matrix since we are measuring states directly
D = np.zeros((3, 2))  # No direct feedthrough

# H-infinity gain matrix 
C_k = np.array([[-102.6001, -0.0000, -0.0000],
                [-0.0000, -678.5667, -800.0]]) #0.5732

# Observer gain matrix (Luenberger Observer)
observer_poles = np.array([-13.9069, -4.3108, -3.5534])  # Chosen poles for observer
L = ctrl.place(A.T, C.T, observer_poles).T

# Initialize the state and state estimate
x_hat = np.zeros((3, 1))  # State estimate: [v; omega]
u_prev = np.zeros((2, 1))  # Previous control inputs: [v; omega]

# Global variables for ROS
start_position = None
current_position = Point()
current_yaw = 0.0
waypoint = Point()
current_velocity = None


def los_guidance(current_position,current_heading,waypoint1,waypoint2,lookahead_distance,velocity,):
    """
    Calculate the desired heading (yaw angle) using LOS guidance.

    Args:
        current_position (tuple): (x, y) coordinates of the current position of the vehicle.
        current_heading (float): Current yaw angle of the vehicle (in radians).
        waypoint1 (tuple): (x_k, y_k) coordinates of the first waypoint.
        waypoint2 (tuple): (x_k1, y_k1) coordinates of the next waypoint.
        lookahead_distance (float): Lookahead distance for the LOS guidance.
        velocity (dict): Contains 'magnitude' for total velocity and 'lateral' for lateral velocity.

    Returns:
        desired_yaw (float): Desired yaw angle to follow the path.
    """

    # Unpack positions
    x = current_position.x
    y = current_position.y
    U = velocity["magnitude"]  # Total velocity (magnitude)
    v = velocity["lateral"]  # Lateral velocity

    # Unpack waypoints
    x_k, y_k = waypoint1.x, waypoint1.y
    x_k1, y_k1 = waypoint2.x, waypoint2.y

    # Calculate the path-tangential angle (alpha_k)
    alpha_k = math.atan2(y_k1 - y_k, x_k1 - x_k)

    # Calculate cross-track error (y_e)
    y_e = -(x - x_k) * math.sin(alpha_k) + (y - y_k) * math.cos(alpha_k)

    # calculate along-track error (x_e) (just for logging)
    x_e = (x - x_k) * math.cos(alpha_k) + (y - y_k) * math.sin(alpha_k)

    # Compute the desired heading (chi_d) using the LOS lookahead algorithm
    chi_d = alpha_k + math.atan2(-y_e, lookahead_distance)

    # Optionally adjust the desired heading for velocity-path angle (beta)
    if U > 0:
        beta = math.asin(v / U)  # Velocity-path relative angle
    else:
        beta = 0  # Avoid division by zero if U is zero

    # Compute the desired yaw angle (desired_yaw)
    desired_yaw = chi_d  # - beta

    return desired_yaw


def switch_waypoint():
    global current_waypoint_index,final_waypoint_reached
    if current_waypoint_index < len(coordinates) - 1:
        current_waypoint_index += 1
        rospy.loginfo(f"Switching to waypoint {current_waypoint_index + 1}")
    else:
        rospy.loginfo("Final waypoint reached.")
        final_waypoint_reached = True

def reset_pid_controllers():
    """ Reset the PID controller errors for linear and angular velocities. """
    linear_pid_nav.err_int = 0
    linear_pid_nav.err_dif = 0
    linear_pid_nav.err_prev = 0
    
    angular_pid_nav.err_int = 0
    angular_pid_nav.err_dif = 0
    angular_pid_nav.err_prev = 0

    rospy.loginfo("PID controllers reset after reaching waypoint.")

def gps_to_enu(lat, lon, alt=0):
    """
    Convert GPS coordinates (lat, lon, alt) to ENU coordinates (x, y, z).

    :param lat: Latitude in degrees
    :param lon: Longitude in degrees
    :param alt: Altitude in meters

    :return x, y, z: ENU coordinates in meters
    """
    # Local coordinate origin (Sydney International Regatta Centre, Australia)
    lat0 = -33.724223  # degree North
    lon0 = 150.679736  # degree East
    alt0 = 0  # meters
    enu = pymap3d.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
    x = enu[0]
    y = enu[1]
    z = enu[2]
    return x, y, z


# Calculate the heading (yaw) from the IMU's orientation quaternion
def calculate_yaw_from_quaternion(orientation):
    # Extract the quaternion values
    qx = orientation.x
    qy = orientation.y
    qz = orientation.z
    qw = orientation.w

    # Calculate yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw


# Callback to update the current IMU data
def imu_callback(msg):
    global current_yaw
    current_yaw = calculate_yaw_from_quaternion(msg.orientation)


# Callback to update the current waypoint
def waypoint_callback(msg):
    global waypoint
    waypoint = msg
    waypoint.x, waypoint.y, waypoint.z = gps_to_enu(msg.x, msg.y)


# Callback to update the current GPS position
def gps_callback(msg):
    global current_position, start_position
    current_position.x = msg.latitude
    current_position.y = msg.longitude
    current_position.x, current_position.y, current_position.z = gps_to_enu(
        msg.latitude, msg.longitude
    )

    # Set start position once when the GPS is first received
    if start_position is None:
        start_position = Point()
        start_position.x = msg.latitude
        start_position.y = msg.longitude
        start_position.x, start_position.y, start_position.z = gps_to_enu(
            msg.latitude, msg.longitude
        )
        rospy.loginfo(
            "Start position set to: (%f, %f)" % (start_position.x, start_position.y)
        )


def velocity_callback(msg):
    global current_velocity
    current_velocity = msg.vector


def calculate_velocity_magnitude():
    global current_velocity

    if current_velocity is not None:
        # Extract surge (x) and lateral (y) velocities
        v_x = current_velocity.x
        v_y = current_velocity.y

        # Compute total velocity magnitude (U)
        U = math.sqrt(v_x**2 + v_y**2)

        # Lateral velocity (v) is just the y component
        v = v_y

        return U, v
    else:
        # Return zeros if velocity hasn't been received yet
        return 0.0, 0.0


# Calculate position error and heading error to the waypoint
def calculate_errors():
    # Calculate the Euclidean distance error to the waypoint
    distance_error = math.sqrt((waypoint.x - current_position.x) ** 2 + (waypoint.y - current_position.y) ** 2)

    # Get U and v from velocity calculations
    U, v = calculate_velocity_magnitude()

    # Create velocity dictionary
    velocity = {"magnitude": U, "lateral": v}

    # Calculate the desired heading to the waypoint
    desired_heading = los_guidance(current_position, current_yaw, start_position, waypoint, LOOKAHEAD_DISTANCE, velocity)
    rospy.loginfo("Desired heading: (%f)" % (desired_heading))

    # Heading error: difference between desired heading and current yaw
    heading_error = desired_heading - current_yaw

    # Normalize heading error to the range [-pi, pi]
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    # Overshoot detection: If the USV is moving away from the waypoint, force it to turn back
    #if is_overshooting_waypoint(current_position, waypoint):
    #    rospy.logwarn("USV overshot waypoint, turning back.")
    #    # Force desired heading directly towards the waypoint
    #    desired_heading = math.atan2(waypoint.y - current_position.y, waypoint.x - current_position.x)
    #    heading_error = desired_heading - current_yaw
    #    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    rospy.loginfo(
        "Heading Error and distance error: (%f,%f)" % (heading_error, distance_error)
    )
    
    return distance_error, heading_error, desired_heading

def is_overshooting_waypoint(current_position, waypoint):
    """
    Detect if the USV is moving away from the waypoint, indicating an overshoot.
    This happens when the distance to the waypoint is increasing rather than decreasing.
    """
    current_distance = math.sqrt((waypoint.x - current_position.x) ** 2 + (waypoint.y - current_position.y) ** 2)
    
    # Compare current distance with previous distance 
    if hasattr(is_overshooting_waypoint, 'previous_distance'):
        if current_distance > is_overshooting_waypoint.previous_distance:
            return True
    
    # Update previous distance
    is_overshooting_waypoint.previous_distance = current_distance
    
    return False

def constrain(input, low, high):
    '''
    Constrain input between lower and higher bounds.
    :param input: Input value to be constrained
    :param low  : Lower bound
    :param high : Higher bound
    :return output: Constrained output value
    '''
    if input < low:
      output = low
    elif input > high:
      output = high
    else:
      output = input
    return output


# Function to calculate thrust from velocities
def calculate_thrust(lin_vel_x, ang_vel_z):
    # Convert the linear and angular velocity into differential thrust commands
    left_thrust = lin_vel_x - ang_vel_z
    right_thrust = lin_vel_x + ang_vel_z
    
    # Constrain the thrust commands
    left_thrust = constrain(left_thrust, -1.0, 1.0)
    right_thrust = constrain(right_thrust, -1.0, 1.0)
    
    return left_thrust, right_thrust

# Helper function to normalize angle to [-pi, pi]
def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Send thruster commands to the WAM-V
def send_thruster_commands(
    left_thrust_pub, right_thrust_pub, left_thrust, right_thrust
):
    # Create Twist messages for both thrusters
    left_thrust_msg = Float32()
    right_thrust_msg = Float32()

    print("Left Thrust: ", left_thrust)
    print("Right Thrust: ", right_thrust)
    print("current_yaw: ", current_yaw)
    # Set data to represent thrust
    left_thrust_msg.data = left_thrust
    right_thrust_msg.data = right_thrust

    # Publish the thruster commands
    left_thrust_pub.publish(left_thrust_msg)
    right_thrust_pub.publish(right_thrust_msg)



def signal_handler(sig, frame):
    rospy.loginfo("Ctrl+C pressed. Saving log data and shutting down...")
    # Save the logged data to a CSV file
    with open('velocity_log.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'Linear Velocity', 'Angular Velocity'])
        for i in range(len(time_log)):
            writer.writerow([time_log[i], lin_vel_log[i], ang_vel_log[i]])
    
    print(f"Writing {len(actual_positions)} entries to path_log.csv")
    # Save the path logs to a CSV file
    with open('path_log.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Actual X', 'Actual Y', 'Expected X', 'Expected Y'])
        for i in range(len(actual_positions)):
            actual_x, actual_y = actual_positions[i]
            expected_x, expected_y = expected_positions[i]
            writer.writerow([actual_x, actual_y, expected_x, expected_y])
    sys.exit(0)


# Differential thrust control loop for waypoint navigation (USING PID)
def differential_thrust_control(dt, current_time):
    global current_waypoint_index, station_keeping_start_time,start_position
    
    # If the final waypoint has been reached, stop the USV
    if final_waypoint_reached:
        rospy.loginfo("Final waypoint reached. Stopping the USV.")
        return 0.0, 0.0  # Zero thrust to stop the USV

    # Set current waypoint based on index
    waypoint.x, waypoint.y = coordinates[current_waypoint_index]
    waypoint.x, waypoint.y,waypoint.z = gps_to_enu(waypoint.x,waypoint.y)

    # Log the expected waypoint position
    expected_positions.append((waypoint.x, waypoint.y))

    # Log the actual current position of the USV
    actual_positions.append((current_position.x, current_position.y))

    # Calculate distance and heading errors
    distance_error, heading_error, waypoint_heading = calculate_errors()

    # Check if we have reached the current waypoint
    if distance_error < distance_threshold:
        # Start station-keeping if not already doing so
        if station_keeping_start_time is None:
            rospy.loginfo(f"Waypoint {current_waypoint_index + 1} reached. Starting station-keeping.")
            reset_pid_controllers()  # Reset PID errors
            station_keeping_start_time = current_time  # Start station-keeping timer

        # Check if station-keeping duration has passed
        if current_time - station_keeping_start_time >= station_keeping_duration:
            rospy.loginfo("Station-keeping complete. Moving to the next waypoint.")
            station_keeping_start_time = None  # Reset station-keeping timer
            start_position = None
            switch_waypoint()  # Move to the next waypoint
            return 0,0
        else:
            # Perform station-keeping (fine-tuning position)
            err_pos = np.array([waypoint.x - current_position.x, waypoint.y - current_position.y])
            rot_tf = normalize_angle(math.atan2(err_pos[1], err_pos[0]) - current_yaw)
            err_pos_local = np.array([np.linalg.norm(err_pos) * math.cos(rot_tf),np.linalg.norm(err_pos) * math.sin(rot_tf)])  # Local frame position error

            # Control linear velocity in the local frame
            lin_vel_x = linear_pid_station.control(err_pos_local[0], current_time)

            # Control angular velocity to match the desired yaw
            err_rot = normalize_angle(waypoint_heading - current_yaw)
            ang_vel_z = angular_pid_station.control(err_rot, current_time)

            #rospy.loginfo("Station-keeping... Linear velocity: %f, Angular velocity: %f",lin_vel_x,ang_vel_z,)

            # Convert linear and angular velocities to differential thrust commands
            left_thrust, right_thrust = calculate_thrust(lin_vel_x, ang_vel_z)

            return left_thrust, right_thrust

    else:
        # Navigating to waypoint
        err_pos = np.array([waypoint.x - current_position.x, waypoint.y - current_position.y])
        lin_vel_x = (np.linalg.norm(err_pos) * v_const)  # Proportional control for linear velocity

        # Limit the linear velocity
        if lin_vel_x > v_limit:
            lin_vel_x = v_limit

        # Calculate the heading error for angular velocity
        #ang_vel_z = angular_pid_nav.control(heading_error, current_time)  # PID for angular velocity
        
        # Set a tolerance for heading error (e.g., 0.05 radians ~ 2.87 degrees)
        heading_tolerance = 0.1

        # Check if the heading error is significant enough to warrant correction
        if abs(heading_error) > heading_tolerance:
            ang_vel_z = angular_pid_nav.control(heading_error, current_time) * (distance_error / distance_threshold)  # PID for angular velocity
        else:
            ang_vel_z = 0  # No correction needed for small heading errors


        #rospy.loginfo("Navigating to waypoint... Linear velocity: %f, Angular velocity: %f",lin_vel_x,ang_vel_z,)

        # Convert linear and angular velocities to differential thrust commands
        left_thrust, right_thrust = calculate_thrust(lin_vel_x, ang_vel_z)

        return left_thrust, right_thrust

# Differential thrust control loop for waypoint navigation (USING HINF for linear and PID for angular velocity)
def differential_thrust_control_hinf_pid(dt, current_time):
    global current_waypoint_index, station_keeping_start_time, start_position, x_hat, u_prev

    # If the final waypoint has been reached, stop the USV
    if final_waypoint_reached:
        rospy.loginfo("Final waypoint reached. Stopping the USV.")
        return 0.0, 0.0  # Zero thrust to stop the USV

    # Set current waypoint based on index
    waypoint.x, waypoint.y = coordinates[current_waypoint_index]
    waypoint.x, waypoint.y, waypoint.z = gps_to_enu(waypoint.x, waypoint.y)

    # Log the expected waypoint position
    expected_positions.append((waypoint.x, waypoint.y))

    # Log the actual current position of the USV
    actual_positions.append((current_position.x, current_position.y))

    # Calculate distance and heading errors
    distance_error, heading_error, waypoint_heading = calculate_errors()

    # Include desired heading (waypoint_heading) in control
    desired_heading = waypoint_heading

    # Measurement vector should include the desired heading
    y_measured = np.array([[distance_error], [heading_error], [desired_heading]])

    # Proportional reduction of linear velocity as the USV approaches the waypoint
    if distance_error < distance_threshold:
        
        # Apply a scaling factor to the control output
        scaling_factor = 0.001  # Adjust this value based on testing
        u_control = -C_k[0, :] @ x_hat  # Compute linear velocity using H-infinity gain matrix
        u_control = u_control.item() * scaling_factor  # Scale down the control output
        
        # Ensure the USV slows down as it reaches the waypoint
        if u_control < 0.1:
            u_control = 0.1  # Minimum velocity to avoid complete stop too early

        rospy.loginfo("Approaching waypoint. Adjusted Linear Velocity: %f", u_control)

        # Continue with station-keeping logic...
        if station_keeping_start_time is None:
            rospy.loginfo(f"Waypoint {current_waypoint_index + 1} reached. Starting station-keeping.")
            station_keeping_start_time = current_time

        if current_time - station_keeping_start_time >= station_keeping_duration:
            rospy.loginfo("Station-keeping complete. Moving to the next waypoint.")
            station_keeping_start_time = None  # Reset station-keeping timer
            start_position = None
            switch_waypoint()
            return 0.0, 0.0  # Zero thrust to stop the USV
        else:
            # H-infinity control for station-keeping (linear velocity only)
            # Update state estimate (Luenberger Observer)
            x_hat_dot = A @ x_hat + B @ u_prev + L @ (y_measured - C @ x_hat)
            x_hat += x_hat_dot * dt

            # Calculate angular velocity using PID controller
            ang_vel_z = angular_pid_station.control(heading_error, current_time)

            rospy.loginfo("Control inputs - Linear Velocity: %f, Angular Velocity (PID): %f", u_control, ang_vel_z)

            # Log data
            time_log.append(current_time)
            lin_vel_log.append(u_control)
            ang_vel_log.append(ang_vel_z)
            
            # Convert velocities to differential thrust
            left_thrust, right_thrust = calculate_thrust(u_control, ang_vel_z)

            # Ensure both control inputs are scalars before storing them
            u_prev = np.array([[u_control], [ang_vel_z]])

            return left_thrust, right_thrust

    else:
        # Apply a scaling factor to the control output
        scaling_factor = 0.01  # Adjust this value based on testing
        u_control = -C_k[0, :] @ x_hat  # Compute linear velocity using H-infinity gain matrix
        u_control = u_control.item() #* scaling_factor  # Scale down the control output
        

        # Update state estimate (Luenberger Observer)
        x_hat_dot = A @ x_hat + B @ u_prev + L @ (y_measured - C @ x_hat)
        x_hat += x_hat_dot * dt

         # Set a tolerance for heading error (e.g., 0.05 radians ~ 2.87 degrees)
        heading_tolerance = 0.1

        # Check if the heading error is significant enough to warrant correction
        if abs(heading_error) > heading_tolerance:
            ang_vel_z = angular_pid_nav.control(heading_error, current_time) * (distance_error / distance_threshold)  # PID for angular velocity
        else:
            ang_vel_z = 0  # No correction needed for small heading errors
        
        # Limit the linear velocity
        if u_control > v_limit:
            u_control = v_limit

        rospy.loginfo("Control inputs - Linear Velocity: %f, Angular Velocity (PID): %f", u_control, ang_vel_z)
        # Log data
        time_log.append(current_time)
        lin_vel_log.append(u_control)
        ang_vel_log.append(ang_vel_z)


        # Convert velocities to differential thrust
        left_thrust, right_thrust = calculate_thrust(u_control, ang_vel_z)

        # Ensure both control inputs are scalars before storing them
        u_prev = np.array([[u_control], [ang_vel_z]])

        return left_thrust, right_thrust


# Main ROS control node
def main():

    rospy.init_node("wamv_differential_thrust_control")

    # Publishers for thruster commands
    left_thrust_pub = rospy.Publisher(LEFT_THRUSTER_TOPIC, Float32, queue_size=10)
    right_thrust_pub = rospy.Publisher(RIGHT_THRUSTER_TOPIC, Float32, queue_size=10)

    # Subscribers for IMU and waypoint
    rospy.Subscriber(IMU_TOPIC, Imu, imu_callback)
    #rospy.Subscriber(WAYPOINT_TOPIC, Point, waypoint_callback)
    rospy.Subscriber(GPS_TOPIC, NavSatFix, gps_callback)
    rospy.Subscriber(FIX_VELOCITY, Vector3Stamped, velocity_callback)

    # Set loop rate
    rate = rospy.Rate(10)  # 10 Hz
    dt = 0.1  # Time step corresponding to the loop rate

    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        # Ensure we have the start position, waypoint, and current position
        if (start_position is not None and waypoint is not None and current_position is not None):
            
            if(CONTROL_PID):
                # Call the differential thrust control function
                left_thrust, right_thrust = differential_thrust_control(dt, current_time)
            else:
                # Call the differential thrust control function
                left_thrust, right_thrust = differential_thrust_control_hinf_pid(dt, current_time)

            # Publish the thruster commands
            send_thruster_commands(left_thrust_pub, right_thrust_pub, left_thrust, right_thrust)

        # Sleep to maintain the loop rate
        rate.sleep()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    try:
        main()
    except rospy.ROSInterruptException:
        pass
