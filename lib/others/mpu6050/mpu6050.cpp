#include "mpu6050.h"

Adafruit_MPU6050 mpu;

void mpu6050_setup(){
  while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
}

MPU6050Values get_mpu6050_value() {
    MPU6050Values values;
    sensors_event_t a, g, temp;

    mpu.getEvent(&a, &g, &temp);

    values.accel_x = a.acceleration.x;
    values.accel_y = a.acceleration.y;
    values.accel_z = a.acceleration.z;
    values.gyro_x = g.gyro.x;
    values.gyro_y = g.gyro.y;
    values.gyro_z = g.gyro.z;

    return values;
}