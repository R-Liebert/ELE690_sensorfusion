#!/usr/bin/env python3

# Import the necessary libraries for reading position as ROS2 Point
import numpy as np
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

# Define the class for subscribing to the camera position topic
class CameraPositionSubscriber(Node):
    def __init__(self):
        super().__init__('camera_position_sub')
        self.subscription = self.create_subscription(Point, 'camera_position', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        global camera_position
        camera_position = msg
        return np.asarray([camera_position.x, camera_position.y, camera_position.z])

# Define the class for subscribing to the imu topic
class imu_subscriber(Node):
    def __init__(self):
        super().__init__('imu_sub')
        self.subscription = self.create_subscription(Imu, 'imu/data', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # get accelerometer data
        accelerometer_data = msg.linear_acceleration
        accelerometer_data = np.asarray([accelerometer_data.x, accelerometer_data.y, accelerometer_data.z])
        # get gyroscope data
        gyroscope_data = msg.angular_velocity
        gyroscope_data = np.asarray([gyroscope_data.x, gyroscope_data.y, gyroscope_data.z])
        return accelerometer_data, gyroscope_data

# Define the class for publishing the position of the object as Point after fusion
class FusionPositionPublisher(Node):
    def __init__(self):
        super().__init__('fusion_position_pub')
        self.publisher_ = self.create_publisher(Point, 'fusion_position', 10)

    def publisher(self, fusion_position):
        msg = Point()
        msg.x = fusion_position[0]
        msg.y = fusion_position[1]
        msg.z = fusion_position[2]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing fusion position: {msg.x}, {msg.y}, {msg.z}')


############################################################################################################
############################################################################################################

# Functions for fusion of camera and imu position
class KalmanFilter:

    '''

    x: vector of state variables [px, py, vx, vy, yaw]
    u: vector of control variables [ax, ay, yaw_rate] (from IMU)
    y: measurement vector [px, py] (from camera)
    A: state transition matrix
    B: control matrix
    C: measurement matrix
    Q: process noise covariance matrix
    R: measurement noise covariance matrix
    P: error covariance matrix
    I: identity matrix 

    '''
    def __init__(self, h=0.2):

        self.h = h
        self.A_c = np.array([[0,0,1,0,0], [0,0,0,1,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]])
        self.B_c = lambda theta: np.array([[0,0,0], [0,0,0], [np.cos(theta),-np.sin(theta),0], [np.sin(theta),np.cos(theta),0], [0,0,1]])
        self.C_d= np.array([[1,0,0,0,0], [0,1,0,0,0]])
        self.Q_d = np.array([[0.1,0,0,0,0], [0,0.1,0,0,0], [0,0,0.1,0,0], [0,0,0,0.1,0], [0,0,0,0,0.1]])
        self.R_d = np.array([[1,0], [0,1]])
        self.I_n = np.eye(len(self.A_c))
        self.A_d = self.I_n + self.A_c * h
        self.B_d = lambda x: self.B_c(x) * self.h


    def filter(self, y, u, x_priori, P_priori):
        KF_Gain = (P_priori[-1]@np.transpose(self.C_d))@np.linalg.inv(self.C_d@P_priori[-1]@np.transpose(self.C_d)+self.R_d)
        P_posteriori = (self.I_n-KF_Gain@self.C_d)@P_priori[-1]@np.transpose(self.I_n-KF_Gain@self.C_d) + KF_Gain@self.R_d@np.transpose(KF_Gain)
        x_posteriori = x_priori[-1] + KF_Gain@(y-self.C_d@x_priori[-1])
        new_x_priori = self.A_d@x_posteriori + self.B_d(x_priori[-1][4])@u
        new_P_priori = self.A_d@P_posteriori@np.transpose(self.A_d) + self.Q_d
        return new_x_priori, new_P_priori, x_posteriori


############################################################################################################
############################################################################################################

def main():
    # Initialise the node
    rclpy.init()

    # Initialise the camera position subscriber
    cam_pos_sub = CameraPositionSubscriber()
    rclpy.spin(cam_pos_sub)
    camera_pos = np.zeros(3)

    # Initialise the imu subscriber
    imu_sub = imu_subscriber()
    rclpy.spin(imu_sub)
    accelerometer_data = np.zeros(3)
    gyroscope_data = np.zeros(3)

    # Initialise the fusion position publisher
    fusion_pos_pub = FusionPositionPublisher()
    rclpy.spin(fusion_pos_pub)
    fusion_pos = np.zeros(3)

    # initial values
    x_initial= np.array([[0], [0], [0], [0], [0]])
    P_initial = np.array([[0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0], [0,0,0,0,0]])
    last_camera_pos = None

    while not rclpy.is_shutdown():
        # Get the camera position
        camera_pos = cam_pos_sub.listener_callback()

        # skip iteration if camera position is unchanged(no new information recieved)
        # IMU data is coming in at a higher frequency than camera data, so we want to 
        # syncronise the data streams to the rate of the camera data
        if camera_pos == last_camera_pos:
            continue

        # Get the imu position
        accelerometer_data, gyroscope_data = imu_sub.listener_callback()

############################################################################################################
############################################################################################################

        # Code for fusion of camera and imu position

        # Fuse the camera and imu position, redneck style
        #fusion_position = (camera_pos + imu_pos)/2

        IMU_message = np.asarray([[accelerometer_data.x], [accelerometer_data.y], [gyroscope_data.z]])
        camera_message = np.asarray([[camera_pos.x], [camera_pos.y]])

        # declaring the vectors as lists
        x_priori = [x_initial]
        P_priori = [P_initial]

        # declaring the Kalman filter
        Kalman = KalmanFilter(h=0.2)
        new_x_priori, new_P_priori, fusion_pos = Kalman.filter(camera_message, IMU_message, x_priori, P_priori)
        
        x_priori.append(new_x_priori)
        P_priori.append(new_P_priori)

        

############################################################################################################
############################################################################################################


        # Publish the fused position
        fusion_pos_pub.publisher(fusion_pos)
        
        # save last camera pos in memory
        last_camera_pos = camera_pos
        # Sleep for 0.5 second
        time.sleep(0.05)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_pos_sub.destroy_node()
    imu_sub.destroy_node()
    fusion_pos_pub.destroy_node()
    rclpy.shutdown()