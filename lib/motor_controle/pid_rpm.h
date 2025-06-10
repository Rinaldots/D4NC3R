#ifndef PID_RPM_H
#define PID_RPM_H

// Declare PID controllers
extern void calculate_pid_left_motor();
extern void calculate_pid_right_motor();
extern int right_motor_min;
extern int left_motor_min;

#endif // PID_RPM_H
