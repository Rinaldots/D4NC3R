#ifndef MOTOR_CONTROLE_H
#define MOTOR_CONTROLE_H

#include <Arduino.h>
#include <std_msgs/msg/int32.h>

//motor A
#define A_IA 25
#define A_IB 26
#define B_IA 33
#define B_IB 32
#define EN_A 27
#define EN_B 14

const int freq = 5000;
const int resolution = 10; // PWM resolution (0-1023 for 10-bit)
const float ENCODER_PULSES_PER_REVOLUTION = 8.0f; // Pulses per revolution for the encoder

// Variáveis de debounce
const unsigned long debounce_delay = 50; // 50 ms de debounce


class MOTOR {
  public:
    // PID Parameters
    float kp, ki, kd;
    float minPwm; // Minimum PWM value to overcome static friction

    // PID State Variables
    float integral;
    float previousError;

    int8_t PIN_A, PIN_B;
    bool inverted;

    // Encoder and Speed Calculation Variables
    volatile long CurrentPosition = 0; // Renamed from Enc_count for consistency in calculateCurrentRpm
    long PreviousPosition = 0; // Made non-volatile, accessed only in calculateCurrentRpm
    unsigned long current_time_2 = 0, previous_time_2 = 0;
    volatile unsigned long encoder_interrupt_time = 0;

    double targetRpm = 0;    // Target RPM for the motor
    double currentRpm = 0;   // Current calculated RPM of the motor
    double pwmOutput = 0;    // Calculated PWM output from PID

    MOTOR(int8_t motor_pin_a, int8_t motor_pin_b, bool is_inverted, float p_gain, float i_gain, float d_gain, float min_pwm_val);
    void pwm(int PWM, bool Direction);
    void stop_motor();
    void calculateCurrentRpm();
    void calculatePid();
}; 

extern MOTOR leftWheel;
extern MOTOR rightWheel;

void motor_setup();

#endif // MOTOR_CONTROLE_H