# Sensorfusion using ROS2 and MicroROS 

In this project NVIDIA Jetson Nano 2GB and Arduino Nano RP2040 Connect is used. The scripts are unfinished as there is issues with initializing camerastream from Pi Camera Module V2 with docker on the Jetson Nano

The goal of the project is to use a kalmannfilter to verify the position of an object detected with the camera using a IMU inside the object. 
To get the IMU data, microROS is used on the Arduino to publish to the \IMU topic using WiFi. This can be found in the ino-file.

As this code was just meant to be used in a report it's not intended for stand-alone use, and the results are just exported to a csv-file for logging purposes.
