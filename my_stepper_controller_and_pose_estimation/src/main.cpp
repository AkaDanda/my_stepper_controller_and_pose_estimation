#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <geometry_msgs/msg/pose.h>
#include <rcl_interfaces/msg/set_parameters_result.h>


#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <WiFi.h>
#include <esp_now.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#include "Stepper.h" 

rcl_subscription_t subscriber;
std_msgs__msg__Int32 received_msg;

rcl_publisher_t publisher;
geometry_msgs__msg__Pose pose_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

const int pin1N1 = 19;
const int pin1N2 = 18;
const int pin1N3 = 14;
const int pin1N4 = 15;  
#define STEPS 360

Stepper stepper(STEPS, pin1N1, pin1N2, pin1N3, pin1N4);

// MAC address of the receiver
uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x54, 0xAA, 0x6C};

// Structure example to send data
typedef struct struct_message {
    int signal;
} struct_message;

struct_message myData;

//Structure to receive data
typedef struct struct_send{
  float qw;
  float qx;
  float qy;
  float qz;
} struct_send;

struct_send myPose;

esp_now_peer_info_t peerInfo;

void send_esp_now_message() {
  // Prepare data to send via ESP-NOW
  myData.signal = 1;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Error sending the message");
  }
}

// Callback function to handle incoming messages on the "keyboard_input" topic
void keyboard_input_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  // Call function to send message via ESP-NOW
  send_esp_now_message();
  
  int angle = 200000;
  stepper.step(angle); // with angle =2048 it makes one rotation
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len == sizeof(struct_send)) {
        // Copia sicura dei dati ricevuti nella struttura myPose
        memcpy(&myPose, data, sizeof(myPose));

        // Popolare il messaggio pose_msg con il quaternione ricevuto
        pose_msg.position.x;
        pose_msg.position.y;
        pose_msg.position.z;
        pose_msg.orientation.w = myPose.qw;
        pose_msg.orientation.x = myPose.qx;
        pose_msg.orientation.y = myPose.qy;
        pose_msg.orientation.z = myPose.qz;

        // Pubblicare il messaggio pose_msg
        RCSOFTCHECK(rcl_publish(&publisher, &pose_msg, NULL));
    } else {
        // Gestire l'errore: lunghezza dei dati non corretta
        printf("Errore: lunghezza dei dati non corretta.\n");
    }
}



void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set the callback function to handle incoming data
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  stepper.setSpeed(35);
  
  // Set the signal pin as output
  pinMode(pin1N1, OUTPUT);
  pinMode(pin1N2, OUTPUT);
  pinMode(pin1N3, OUTPUT);
  pinMode(pin1N4, OUTPUT);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Keyboard_input"));

   // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
    "pose_communication"));


  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &keyboard_input_callback, ON_NEW_DATA));
}

void loop() {
  delay(1000);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  Serial.println("Failed to add peer");
}

