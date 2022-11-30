#!/usr/bin/env python3

# Impoor the necessary libraries for reading raspberry pi camera on jetson nano
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Define the class for publishing the image
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_pub')
        self.publisher_ = self.create_publisher(Image, 'image', 10)

    def publisher(self, img):
        self.publisher_.publish(img)
        self.get_logger().info('Publishing image')


def main():
    # Initialize the video capture from gstreamer

    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    _, img = cap.read()

    rclpy.init()
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)


    while not rclpy.is_shutdown() and cap.isOpened():
        #read the image from the camera on /dev/video0
        _, img = cap.read()
        img = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        image_publisher.publisher(img=img)
        time.sleep(0.05)

    cap.release()
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()