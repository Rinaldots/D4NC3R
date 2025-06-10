#ifndef LED_COMUN_H
#define LED_COMUN_H

#include <Arduino.h>
#include <WiFi.h>
#include <string>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <ShiftRegister74HC595.h>

#define LED_PIN 2
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn;if (temp_rc != RCL_RET_OK) { while (1){digitalWrite(LED_PIN,!digitalRead(LED_PIN));delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){ Serial.print("Error in function, code: "); Serial.println(temp_rc); } }
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define CONCAT(a, b, c) a b c

extern float wheels_y_distance_;
extern float wheel_radius;
extern float wheel_circumference_;

#include "motor_controle.h"

#include "../others/mpu6050/mpu6050.h"
// #include "odometry.h" // Removido pois o publicador de odometria será removido

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int32_multi_array.h> // Adicionado para o novo publicador de encoders
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <micro_ros_utilities/string_utilities.h>

// Topicos
#define D4NC3RNUMBER 1
#define NAMESPACE CONCAT("/d4nc3r", TOSTRING(D4NC3RNUMBER), "")
#define TOPIC_LED CONCAT(NAMESPACE,"/led", "")
#define TOPIC_DISPLAY CONCAT(NAMESPACE,"/display", "")
#define TOPIC_MPU6050 CONCAT(NAMESPACE,"/imu", "")
#define TOPIC_MOTOR CONCAT(NAMESPACE,"/cmd_vel", "")
#define TOPIC_ENCODERS CONCAT(NAMESPACE, "/encoders", "") // Novo tópico para os dados dos encoders
// #define TOPIC_ODOM CONCAT("micro_ros", "","/odom/unfiltered") // Removido


extern rcl_node_t node;
extern rclc_support_t support;
extern rcl_allocator_t allocator;

extern rcl_subscription_t subscriber_motor;


//Publisher declaration
extern rclc_executor_t executor_motor_sub;
extern rcl_publisher_t publisher_mpu6050;
extern rcl_publisher_t encoders_publisher; // Novo publicador de encoders

//Message declaration
extern std_msgs__msg__Int32 msg_led;
extern std_msgs__msg__Int8 msg_display;
extern std_msgs__msg__Int32MultiArray encoders_msg; // Nova mensagem para os encoders
extern sensor_msgs__msg__Imu mpu6050_msg;

// Declare the shared functions
void ros_setup();

void ros2_loop();


void publish_data();
void led_subiscriber();
std::string padString(const std::string &input);
void subscription_callback(const void *msgin);

void motor_subiscriber();
void subscription_motor_callback(const void *msgin);

void display_subiscriber();
void display_sub_callback(const void *msgin);

void mpu_publisher_setup();

void MotorControll_callback(const void *msgin);

void publisher_setup();


// Funções de sincronização de tempo
void syncTime();
struct timespec getTime();

extern unsigned long long time_offset;
#endif // LED_COMUN_H