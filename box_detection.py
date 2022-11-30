#!/usr/bin/env python3

# Import the necessary libraries
import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point


# Define the class for subscribing to the image topic 
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')
        self.subscription = self.create_subscription(Image, 'image', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        return img

# Define the class for publishing center of bounding box as Point
class BoxPublisher(Node):
    def __init__(self):
        super().__init__('box_pub')
        self.publisher_ = self.create_publisher(Point, 'camera_position', 10)

    def publisher(self, box_center):
        msg = Point()
        msg.x = box_center[0]
        msg.y = box_center[1]
        msg.z = 0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing bounding box center: "{msg.x}, {msg.y}"')



class camera_distance:

    '''
    
    Inputs: 
        - img: image
        - bounding_box: bounding box of object

        - focal_length: focal length of camera
        - sensor_size: sensor size of camera

    Outputs:
        - distance x: off-centre distance from the camera [mm]
        - distance y: distance to the object from the camera [mm]
    
    '''
    #define init parameters
    def __init__(self,
                bounding_box,
                focal_length = 3.04,
                sensor_size = [3.68, 2.76],
                resolution = [1920, 1080],
                method = "vertical",
                object_width_mm = 130):
        
        #init parameters
        self.method = method

        #camera parameters
        self.focal_length = focal_length
        self.sensor_size = sensor_size
        self.resolution = resolution

        #object paramaeters
        self.bounding_box = bounding_box
        self.object_width_mm = object_width_mm
        self.object_width_pixels = tuple(bounding_box[2])

        #image parameters
        self.pixels_per_mm = self.object_width_pixels / self.object_width_mm
    
    def distance_y_vertical(self):
        object_width_on_sensor = (self.object_width_pixels * self.sensor_size[1]) / self.resolution[0]
        distance_y = (self.object_width_mm * self.focal_length) / (object_width_on_sensor)
        return np.float32(distance_y)

    def distance_y_horizontal(self):
        object_width_on_sensor = (self.object_width_pixels * self.sensor_size[0]) / self.resolution[1]
        distance_y = (self.object_width_mm * self.focal_length) / (object_width_on_sensor)
        return np.float32(distance_y)

    def distance_x_vertical(self):
        object_centre = tuple(self.bounding_box[0])
        image_centre = self.resolution[0] / 2
        distance_x = (object_centre - image_centre) / self.pixels_per_mm
        return np.float32(distance_x)      

    def distance_x_horizontal(self):
        object_centre = tuple(self.bounding_box[0])
        image_centre = self.resolution[1] / 2
        distance_x = (object_centre - image_centre) / self.pixels_per_mm
        return np.float32(distance_x)      

    def find_distance(self):
        if self.method == "vertical":
            distance_x = self.distance_x_vertical()
            distance_y = self.distance_y_vertical()
        elif self.method == "horizontal":
            distance_x = self.distance_x_horizontal()
            distance_y = self.distance_y_horizontal()
        else:
            print("Method not recognised")
        return distance_x, distance_y  


def get_bounding_box(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # define range of green color in HSV
    lower_green = np.array([40,50,50])
    upper_green = np.array([100,255,255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Find contours 
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Find the largest contour
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
        bounding_box = np.asarray([x, y, w, h], dtype=np.int32)
    
    #cv2.polylines(img, [bounding_box], True, (255,0,0), 15)
    return bounding_box

def main():
    rclpy.init()

    img_sub = ImageSubscriber()
    rclpy.spin(img_sub)

    box_pub = BoxPublisher()
    rclpy.spin(box_pub)
    last_bound_box = None

    while not rclpy.is_shutdown():
        bounding_box = get_bounding_box(img_sub.listener_callback())

        if bounding_box == last_bound_box:
            continue
        
        instance = camera_distance(bounding_box)
        
        x_mm, y_mm = instance.find_distance()
        box_center = (x_mm, y_mm)

        box_pub.publisher(box_center)

        last_bound_box = bounding_box

    img_sub.destroy_node()
    box_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()