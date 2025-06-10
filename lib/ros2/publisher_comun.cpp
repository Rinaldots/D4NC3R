#include "ros2.h"

unsigned long prev_odom_update = 0;
unsigned long prev_mpu_update = 0;

#define TICKS_PER_REVOLUTION 8

rcl_publisher_t odom_publisher;
Odometry odometry;
nav_msgs__msg__Odometry odom_msg;

rcl_publisher_t publisher_mpu6050;
sensor_msgs__msg__Imu mpu6050_msg;

void odom_publisher_setup() {
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    TOPIC_ODOM));
  Serial.println("Odometry publisher created");
}

void mpu_publisher_setup() {
  RCCHECK(rclc_publisher_init_default(
    &publisher_mpu6050, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    TOPIC_MPU6050));
  Serial.println("Mpu publisher created");
}

void publish_mpu6050() {
  MPU6050Values mpu_value = get_mpu6050_value();
  struct timespec time_stamp = getTime();
  mpu6050_msg.header.frame_id = micro_ros_string_utilities_set(mpu6050_msg.header.frame_id, "mpu_link");
  mpu6050_msg.header.stamp.sec = time_stamp.tv_sec;
  mpu6050_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  mpu6050_msg.linear_acceleration.x = mpu_value.accel_x;
  mpu6050_msg.linear_acceleration.y = mpu_value.accel_y;
  mpu6050_msg.linear_acceleration.z = mpu_value.accel_z;
  mpu6050_msg.angular_velocity.x = mpu_value.gyro_x;
  mpu6050_msg.angular_velocity.y = mpu_value.gyro_y;
  mpu6050_msg.angular_velocity.z = mpu_value.gyro_z;
  RCSOFTCHECK(rcl_publish(&publisher_mpu6050, &mpu6050_msg, NULL));
}

void publish_odom() {
  struct timespec time_stamp = getTime();
  unsigned long now = millis();
  float rpmL = leftWheel.Velocidade_dv;
  float rpmR = rightWheel.Velocidade_dv;
  float avg_rps_x = (rpmL + rpmR) * 0.5f / 20.0f;
  float linear_x = avg_rps_x * wheel_circumference_;
  float avg_rps_a = (-rpmL + rpmR) * 0.5f / 20.0f;
  float angular_z = (avg_rps_a * wheel_circumference_) / (wheels_y_distance_ * 0.5f);
  float vel_dt = (now - prev_odom_update) * 0.001f;
  prev_odom_update = now;
  odometry.update(vel_dt, linear_x, 0, angular_z);
  odom_msg = odometry.getData();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void publish_data() {

  if(millis() - prev_mpu_update > (1000/10)){
    prev_mpu_update = millis();
    publish_mpu6050();
  }
  if (millis() - prev_odom_update > (1000/15)) {
    prev_odom_update = millis();
    publish_odom();
  }
  //Serial.println("Published published");
}


void publisher_setup() {
  odom_publisher_setup();
  mpu_publisher_setup();
  Serial.println("Publisher created");
}