#include <micro_ros_arduino.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

#if !defined(ARDUINO_NANO_RP2040_CONNECT)
#error "This code is only written for Arduino Nano RP2040 Connect"
#endif

char ssid[] = "ice.net-711B5B";
char pass[] = "E12B58E1";
int status = WL_IDLE_STATUS;

float Ax, Ay, Az;
float Gx, Gy, Gz;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Setup loop for communicating errors visualy
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Print WiFi Status over serial
void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  //print board ip
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// Setup WiFi connection, serial for debugging and microROS WiFi.
// Activate IMU and prepare ROS2 Publisher
void setup() {
  Serial.begin(9600);
  
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while(true);  
  }
  while (status != WL_CONNECTED){
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  
  printWiFiStatus();
  
  set_microros_wifi_transports("ice.net-711B5B", "E12B58E1", "192.168.0.7", 8888);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while(1);
  }
  if (IMU.accelerationAvailable()){
      IMU.readAcceleration(Ax, Ay, Az);
  }
  Serial.print("Got Ax: ");
  Serial.println(Ax);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();
  Serial.println("Got Allocator");
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("Created support init");
  // create node
  RCCHECK(rclc_node_init_default(&node, "imu_publisher_node", "", &support));
  Serial.println("Created node");

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu"));
  Serial.println("Created publisher");
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // initialize message
  sensor_msgs__msg__Imu__init(&imu_msg);

  digitalWrite(LED_PIN, LOW);

}

// Read IMU data in G's and convert to m/s^2 before publishing
// to /imu topic every 10ms
void loop() {
  // read imu data
    if (IMU.accelerationAvailable()){
      IMU.readAcceleration(Ax, Ay, Az);
      imu_msg.linear_acceleration.x = Ax*9.80665;
      imu_msg.linear_acceleration.y = Ay*9.80665;
      imu_msg.linear_acceleration.z = Az*9.80665;
    }
    if (IMU.gyroscopeAvailable()){
      IMU.readGyroscope(Gx, Gy, Gz);
      imu_msg.angular_velocity.x = Gx*0.017453;
      imu_msg.angular_velocity.y = Gy*0.017453;
      imu_msg.angular_velocity.z = Gz*0.017453;
    }
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    delay(10);
}