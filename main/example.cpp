/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "sdkconfig.h"
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PIN_SDA 14
#define PIN_CLK 2

Quaternion q;             // [w, x, y, z]         quaternion container
VectorFloat gravity;      // [x, y, z]            gravity vector
float ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU

void task_initI2C(void *ignore) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)PIN_SDA;
  conf.scl_io_num = (gpio_num_t)PIN_CLK;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.clk_stretch_tick = 300;
  // conf.master.clk_speed = 400000;
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode));
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  vTaskDelete(NULL);
}

void task_display(void *) {
  MPU6050 mpu = MPU6050();
  mpu.initialize();
  while (!mpu.testConnection()) {
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }

  uint8_t dmpStatus = mpu.dmpInitialize();
  ESP_LOGI("MPU6050", "DMP initialization code:%d", dmpStatus);

  // This need to be setup individually
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  // mpu.PrintActiveOffsets();

  mpu.setDMPEnabled(true);

  mpuIntStatus = mpu.getIntStatus();
  ESP_LOGE("MPU6050", "=============> %d", mpuIntStatus);

  packetSize = mpu.dmpGetFIFOPacketSize();
  printf("%d\n", packetSize);

  while (1) {
    if (mpu.GetCurrentFIFOPacket(fifoBuffer, packetSize)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      printf("YAW: %3.1f, ", ypr[0] * 180 / M_PI);
      printf("PITCH: %3.1f, ", ypr[1] * 180 / M_PI);
      printf("ROLL: %3.1f \n", ypr[2] * 180 / M_PI);
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}
