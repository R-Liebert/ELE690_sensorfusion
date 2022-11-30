#!/usr/bin/env python3

import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

# Define the class for publishing the position as a Point
class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_pub')
        self.publisher_ = self.create_publisher(Point, 'imu_position', 10)

    def publisher(self, position_vector):
        msg = Point()
        # get last three elements of position vector 2d-array
        msg.x = position_vector[0, -1]
        msg.y = position_vector[1, -1]
        msg.z = 0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.x}. {msg.y}. {msg.z}')

# Define the class for subscribing to the imu topic setting QoS to best effort
class imu_subscriber(Node):
    def __init__(self):
        super().__init__('imu_sub')
        self.subscription = self.create_subscription(Imu, 'imu', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # get accelerometer data
        accelerometer_data = msg.linear_acceleration
        accelerometer_data = np.asarray([accelerometer_data.x, accelerometer_data.y, accelerometer_data.z])
        # get gyroscope data
        gyroscope_data = msg.angular_velocity
        gyroscope_data = np.asarray([gyroscope_data.x, gyroscope_data.y, gyroscope_data.z])
        return accelerometer_data, gyroscope_data

# Define the class for subscribing to the camera_position topic as a Point
class camera_position_subscriber(Node):
    def __init__(self):
        super().__init__('camera_position_sub')
        self.subscription = self.create_subscription(Point, 'camera_position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        camera_position = np.asarray([msg.x, msg.y, msg.z])
        return camera_position


def find_position_(accelerometer_data, gyroscope_data, position_vector, velocity_vector, orientation_vector, time_step=0.2):
    accelerometer_data = np.asarray([accelerometer_data[1], -accelerometer_data[0]])
    gyroscope_data = gyroscope_data[2]

    # Velocity from acceleration
    find_velocity = lambda previous_velocity, acceleration, time_step: np.asarray(previous_velocity) + np.asanyarray(acceleration) * time_step
    
    # Define a rotation matrix from body to world frame
    rotation_matrix = lambda yaw, array: np.reshape(np.asarray([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]), (2,2)) @ array

    # Find next velocity and avergae velocity between the time steps
    next_velocity = find_velocity(velocity_vector[-1], accelerometer_data, time_step)
    avg_speed_frame = (velocity_vector[-1] + next_velocity) / 2

    # Find next orientation and average orientation between the time steps
    next_orientation = orientation_vector[-1] + gyroscope_data * time_step

     # Distance traveled regardless of direction
    temp = np.reshape(avg_speed_frame * time_step, (2,1))

    # Rotate the distance traveled to the world frame, add to previous position
    position_frame = position_vector[-1] + rotation_matrix(next_orientation, temp)

     # Update the vectors
    velocity_vector.append(next_velocity)
    position_vector.append(position_frame)
    orientation_vector.append(next_orientation)

    return position_vector, velocity_vector, orientation_vector



def main():
    rclpy.init()

    position_vector = [np.asarray([0, 0, 0])]
    # create a subscriber to subscribe to the imu topic
    imu_sub = imu_subscriber()

    # Create a publisher to publish the position
    position_pub = PositionPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu_sub)
    executor.add_node(position_pub)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # create a subscriber to subscribe to the bounding box topic
    try:
        cam_pos_sub = camera_position_subscriber()
        rclpy.spin(cam_pos_sub)
    except Exception:
        pass
    
    # Initialize the position vector and wait for the camera position to be published
    initial_position = np.asarray([0,0,0])
    i = 0
    while cam_pos_sub.listener_callback() is None and i < 50:
        i += 1
        time.sleep(0.1)
    
    # Set initial position and values of IMU from camera capture if available
    if cam_pos_sub.listener_callback() is not None:
        initial_position = cam_pos_sub.listener_callback()
        initial_position = np.asarray([initial_position[0], initial_position[1], initial_position[2]])
    
    initial_velocity = np.asarray([0, 0])
    initial_orientation = np.asarray([0])

    position_vector = [initial_position]
    velocity_vector = [initial_velocity]
    orientation_vector = [initial_orientation]

    # get initial imu data
    while imu_sub.listener_callback() is None:
        time.sleep(0.1)

    accelerometer_data, gyroscope_data = imu_subscriber.listener_callback(imu_sub)

    # find and publish position
    while rclpy.ok():
        accelerometer_data, gyroscope_data = imu_subscriber.listener_callback(imu_sub)
        position_vector, velocity_vector, orientation_vector = find_position_(accelerometer_data, gyroscope_data, position_vector, velocity_vector, orientation_vector, time_step=1)
        position_pub.publisher(position_vector=position_vector)

    position_pub.destroy_node()
    imu_sub.destroy_node()
    try :
        cam_pos_sub.destroy_node()
    except Exception:
        pass
    executor.shutdown()
    rclpy.shutdown()


    

if __name__ == "__main__":
    main()