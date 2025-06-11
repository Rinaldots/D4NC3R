#include "ros2.h"

// Variáveis de controle de tempo para publicação
unsigned long prev_mpu_update = 0;
unsigned long prev_encoders_update = 0; // Para o novo publicador de encoders
// #define TICKS_PER_REVOLUTION 8 // Não é mais necessário aqui se publicamos ticks brutos

// Defina as taxas de publicação desejadas em Hz
#define MPU_PUBLISH_RATE_HZ 15
#define ENCODERS_PUBLISH_RATE_HZ 50

// Calcule os intervalos de publicação em milissegundos
const unsigned long MPU_PUBLISH_INTERVAL_MS = 1000UL / MPU_PUBLISH_RATE_HZ;
const unsigned long ENCODERS_PUBLISH_INTERVAL_MS = 1000UL / ENCODERS_PUBLISH_RATE_HZ;

// Publicador e mensagem para MPU6050 (IMU)
rcl_publisher_t publisher_mpu6050;
sensor_msgs__msg__Imu mpu6050_msg;

// Publicador e mensagem para os Encoders
rcl_publisher_t encoders_publisher;
std_msgs__msg__Int32MultiArray encoders_msg;
// Buffers estáticos para os dados da mensagem Int32MultiArray para evitar alocações dinâmicas no loop
static int32_t encoder_data_buffer[2]; // [left_ticks, right_ticks]
static std_msgs__msg__MultiArrayDimension encoder_layout_dim_buffer[1];


void encoders_publisher_setup() {
  RCCHECK(rclc_publisher_init_best_effort(
    &encoders_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    TOPIC_ENCODERS));
  
  // Inicializa a mensagem e configura seu layout e dados
  std_msgs__msg__Int32MultiArray__init(&encoders_msg);
  encoders_msg.data.data = encoder_data_buffer;
  encoders_msg.data.capacity = 2;
  encoders_msg.data.size = 2; // Sempre teremos 2 valores (encoder esquerdo e direito)

  std_msgs__msg__MultiArrayDimension__init(&encoder_layout_dim_buffer[0]);
  encoder_layout_dim_buffer[0].label = micro_ros_string_utilities_set(encoder_layout_dim_buffer[0].label, "wheel_ticks");
  encoder_layout_dim_buffer[0].size = 2;
  encoder_layout_dim_buffer[0].stride = 2; // Para um array 1D com 2 elementos

  encoders_msg.layout.dim.data = encoder_layout_dim_buffer;
  encoders_msg.layout.dim.capacity = 1;
  encoders_msg.layout.dim.size = 1;
  encoders_msg.layout.data_offset = 0;

  Serial.println("Encoders publisher created (Int32MultiArray)");
}


void mpu_publisher_setup() {
  RCCHECK(rclc_publisher_init_best_effort(
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
  // ... (atribuições de valores da mpu_value para mpu6050_msg) ...
  mpu6050_msg.linear_acceleration.x = mpu_value.accel_x;
  mpu6050_msg.linear_acceleration.y = mpu_value.accel_y;
  mpu6050_msg.linear_acceleration.z = mpu_value.accel_z;
  mpu6050_msg.angular_velocity.x = mpu_value.gyro_x;
  mpu6050_msg.angular_velocity.y = mpu_value.gyro_y;
  mpu6050_msg.angular_velocity.z = mpu_value.gyro_z;
  
  RCSOFTCHECK(rcl_publish(&publisher_mpu6050, &mpu6050_msg, NULL));
  
}

void publish_encoders() {
  struct timespec time_stamp = getTime();
  // Lê os valores atuais dos encoders (são volatile long)
  int32_t left_ticks = leftWheel.CurrentPosition;
  int32_t right_ticks = rightWheel.CurrentPosition;

  encoders_msg.data.data[0] = left_ticks;
  encoders_msg.data.data[1] = right_ticks;
  encoders_msg.data.size = 2; // Garante que o tamanho está correto

  RCSOFTCHECK(rcl_publish(&encoders_publisher, &encoders_msg, NULL));
}

void publish_data() {
  unsigned long current_millis = millis();

  // Publicação do MPU6050
  if (current_millis - prev_mpu_update > MPU_PUBLISH_INTERVAL_MS) {
    prev_mpu_update = current_millis; // Registra o tempo antes da publicação
    publish_mpu6050();
  }

  // Re-obtém o tempo atual, pois publish_mpu6050() pode ter demorado
  current_millis = millis();

  // Publicação dos Encoders
  if (current_millis - prev_encoders_update > ENCODERS_PUBLISH_INTERVAL_MS) {
    prev_encoders_update = current_millis; // Registra o tempo antes da publicação
    publish_encoders();
  }
}

void publisher_setup() {
  encoders_publisher_setup(); // Configura o novo publicador de encoders
  mpu_publisher_setup();
  Serial.println("Publisher created");
}