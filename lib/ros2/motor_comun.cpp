#include "ros2.h"
#include <geometry_msgs/msg/twist.h>

#include "motor_controle.h"
rcl_subscription_t subscriber_motor;
rclc_executor_t executor_motor_sub;
geometry_msgs__msg__Twist msg_motor;

unsigned long prev_motor_update = 0;

void motor_subiscriber(){
  RCCHECK(rclc_subscription_init_default(&subscriber_motor, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_MOTOR));
  RCCHECK(rclc_executor_add_subscription(&executor_motor_sub, &subscriber_motor, &msg_motor, &MotorControll_callback, ALWAYS)); 
  Serial.println("Motor subscriber created");
}


void MotorControll_callback(const void* msg_in) {
  //Serial.println("Motor control callback triggered");
  
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
    float linearVelocity = msg->linear.x;
    float angularVelocity = msg->angular.z;

    float vL = linearVelocity - (angularVelocity * 0.5f);
    float vR = linearVelocity + (angularVelocity * 0.5f);

    leftWheel.targetRpm = vL / (wheel_circumference_ / 60);
    rightWheel.targetRpm = vR / (wheel_circumference_ / 60);

    prev_motor_update = millis();
    leftWheel.calculateCurrentRpm();
    rightWheel.calculateCurrentRpm();

    Serial.print("Left Wheel Target RPM: ");
    Serial.println(leftWheel.targetRpm);
    Serial.print("Right Wheel Target RPM: ");
    Serial.println(rightWheel.targetRpm);
    Serial.print("Left Wheel Current RPM: ");
    Serial.println(leftWheel.currentRpm);
    Serial.print("Right Wheel Current RPM: ");
    Serial.println(rightWheel.currentRpm);


    if(leftWheel.targetRpm <= 0){
      leftWheel.pwm(0, 1);
      leftWheel.pwmOutput = 0;
    }
    else{
      leftWheel.calculatePid();
      Serial.println("Left Wheel PWM Output: " + String(leftWheel.pwmOutput));
      leftWheel.pwm(leftWheel.pwmOutput, 1);
    }
    if(rightWheel.targetRpm <= 0){
      rightWheel.pwm(0, 1);
      rightWheel.pwmOutput = 0;
    }
    else{
      rightWheel.calculatePid();
      Serial.println("Right Wheel PWM Output: " + String(rightWheel.pwmOutput));
      rightWheel.pwm(rightWheel.pwmOutput, 1);
    }
  

}



