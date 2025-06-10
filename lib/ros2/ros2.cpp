#include "ros2.h"
#include <micro_ros_platformio.h>


rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
unsigned long long time_offset = 0;
float wheels_y_distance_ = 0.20;
float wheel_radius = 0.035;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;
rcl_timer_t timer;
rclc_executor_t executor_pub;


void syncTime() {
    rmw_uros_sync_session(1000);
    unsigned long now = millis();
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime() {
    struct timespec tp = { 0 };
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}


void ros_setup() {    
    pinMode(LED_PIN, OUTPUT);

    IPAddress agent_ip(192, 168, 1, 123);
    size_t agent_port = 8888;
    char ssid[] = "10D";
    char psk[]= "12345678";
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "baile", NAMESPACE, &support));

    // Inicialize o executor para subscribers
    RCCHECK(rclc_executor_init(&executor_motor_sub, &support.context, 1, &allocator));


    motor_subiscriber();

    syncTime(); // Sincronize o tempo antes de iniciar o timer

    publisher_setup(); // publisher_setup chama timer_setup, que inicializa e adiciona o timer ao executor_pub
}

void ros2_loop() {

    RCCHECK(rclc_executor_spin_some(&executor_motor_sub, 66666666)); // 15 Hz in nanoseconds

    publish_data();

    leftWheel.Calcular_Velocidade();
    rightWheel.Calcular_Velocidade();
    /*Serial.print("Left/ RPM: ");
    Serial.print(leftWheel.Velocidade_dv);
    Serial.print(" | Right RPM: ");
    Serial.print(rightWheel.Velocidade_dv);
    
    Serial.print("  / Left PWM Value: ");
    Serial.print(leftWheel.PWM_value);
    Serial.print(" | Right PWM Value: ");  
    Serial.println(rightWheel.PWM_value);
    */
   

    if(leftWheel.RPM_target <= 0){
      leftWheel.pwm(0, 1);
      leftWheel.PWM_value = 0;
    }
    else{
      calculate_pid_left_motor();
      leftWheel.pwm(leftWheel.PWM_value, 1);
    }
    if(rightWheel.RPM_target <= 0){
      rightWheel.pwm(0, 1);
      leftWheel.PWM_value = 0;
    }
    else{
      calculate_pid_right_motor();
      rightWheel.pwm(rightWheel.PWM_value, 1);
    }
}

