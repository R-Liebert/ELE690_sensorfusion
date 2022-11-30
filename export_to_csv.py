#!/usr/bin/env python3

# Import the necessary libraries for reading ros2 vector topic and writing to csv file
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from geometry_msgs.msg import Point
import csv

# Define the class for subscribing to the Fusion Position topic as Point
class FusionPositionSubscriber(Node):
    def __init__(self):
        super().__init__('fusion_position_sub')
        self.subscription = self.create_subscription(Point, 'fusion_position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global fusion_position
        fusion_position = msg
        return np.asarray([fusion_position.x, fusion_position.y, fusion_position.z])

# Class for subscribing to imu position topic
class ImuPositionSubscriber(Node):
    def __init__(self):
        super().__init__('imu_position_sub')
        self.subscription = self.create_subscription(Point, 'imu_position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global imu_position
        imu_position = msg
        return np.asarray([imu_position.x, imu_position.y, imu_position.z])

# Class for subscribing to camera position topic
class CameraPositionSubscriber(Node):
    def __init__(self):
        super().__init__('camera_position_sub')
        self.subscription = self.create_subscription(Point, 'camera_position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global camera_position
        camera_position = msg
        return np.asarray([camera_position.x, camera_position.y, camera_position.z])


# Define the class for writing an new line to csv file
class CSVWriter():
    def __init__(self):
        super().__init__('csv_writer')
        self.writer = csv.writer(open('data.csv', 'a', newline=''))

    def writer(self, data):
        self.writer.writerow(data)
        self.get_logger().info('Writing to csv file')

def main():
    rclpy.init()

    # Create the node for subscribing to the fusion position topic
    fusion_pos_sub = FusionPositionSubscriber()

    # Create the node for subscribing to the imu position topic
    imu_pos_sub = ImuPositionSubscriber()
    rclpy.spin(imu_pos_sub)


    # Create the node for subscribing to the camera position topic
    camera_pos_sub = CameraPositionSubscriber()

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(fusion_pos_sub)
        executor.add_node(camera_pos_sub)
        # Spin in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
    except Exception:
        pass

    rclpy.spin(imu_pos_sub)

    fusion_pos = np.zeros(3)
    imu_pos = np.zeros(3)
    camera_pos = np.zeros(3)

    # Create the node for writing to csv file
    csv_writer = CSVWriter()

    while not rclpy.is_shutdown():
        fusion_pos = fusion_pos_sub.listener_callback()
        imu_pos = imu_pos_sub.listener_callback()

        # Chech if camera position is not zero
        if camera_pos_sub.listener_callback() is not None:
            camera_pos = camera_pos_sub.listener_callback()
            
        # write to csv file
        csv_writer.writer(f"Position estimated by fusion: {fusion_pos} \nPosition estimated by imu: {imu_pos} \nPosition estimated by camera: {camera_pos}")
        time.sleep(0.5)

    fusion_pos_sub.destroy_node()
    csv_writer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
