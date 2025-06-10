#include "pid_rpm.h"
#include "motor_controle.h" // Include the header where Wheel is defined

// PID parameters for left and right motors
float kp_left = 1, ki_left = 0.1, kd_left = 0.002;
float kp_right = 1, ki_right = 0.1, kd_right = 0.002;

// PID state variables for left motor
float integral_left = 0;
float previous_error_left = 0;

// PID state variables for right motor
float integral_right = 0;
float previous_error_right = 0;

int left_motor_min = 450;
int right_motor_min = 450;

// Custom PID function for left motor
void calculate_pid_left_motor() {
    float error = leftWheel.RPM_target - leftWheel.Velocidade_dv;
    integral_left += error;
    float derivative = error - previous_error_left;
    leftWheel.PWM_value = kp_left * error + ki_left * integral_left + kd_left * derivative+left_motor_min;

    // Clamp PWM value to valid range
    if (leftWheel.PWM_value > 1023) leftWheel.PWM_value = 1023;
    if (leftWheel.PWM_value < left_motor_min+1) leftWheel.PWM_value = 0;

    previous_error_left = error;
}

// Custom PID function for right motor
void calculate_pid_right_motor() {
    float error = rightWheel.RPM_target - rightWheel.Velocidade_dv;
    integral_right += error;
    float derivative = error - previous_error_right;
    rightWheel.PWM_value = kp_right * error + ki_right * integral_right + kd_right * derivative + right_motor_min;

    // Clamp PWM value to valid range
    if (rightWheel.PWM_value > 1023) rightWheel.PWM_value = 1023;
    if (rightWheel.PWM_value < right_motor_min+1) rightWheel.PWM_value = 0;

    previous_error_right = error;
}
