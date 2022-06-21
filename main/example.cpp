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

#define PIN_SDA 12
#define PIN_CLK 14

MPU6050 mpu;
Quaternion q;             // [w, x, y, z]         quaternion container
VectorFloat gravity;      // [x, y, z]            gravity vector
float ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int32_t accel[3];         // [x, y, z]            accel vector
int16_t accel_raw[3];     // [x, y, z]            raw accelerometer vector
int16_t grav[3];          // [x, y, z]
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

void printfln(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  printf("\n");
}

void checkSettings() {

  printf(" * Sleep Mode:                ");
  printfln(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  printf(" * Motion Interrupt:          ");
  printfln(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

  printf(" * Zero Motion Interrupt:     ");
  printfln(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

  // printf(" * Free Fall Interrupt:       ");
  // printfln(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

  printf(" * Motion Threshold:          ");
  printfln("%d", mpu.getMotionDetectionThreshold());

  printf(" * Motion Duration:           ");
  printfln("%d", mpu.getMotionDetectionDuration());

  printf(" * Zero Motion Threshold:     ");
  printfln("%d", mpu.getZeroMotionDetectionThreshold());

  printf(" * Zero Motion Duration:      ");
  printfln("%d", mpu.getZeroMotionDetectionDuration());

  printf(" * Clock Source:              ");
  switch (mpu.getClockSource()) {
  case MPU6050_CLOCK_KEEP_RESET:
    printfln("Stops the clock and keeps the timing generator in reset");
    break;
  case MPU6050_CLOCK_PLL_EXT19M:
    printfln("PLL with external 19.2MHz reference");
    break;
  case MPU6050_CLOCK_PLL_EXT32K:
    printfln("PLL with external 32.768kHz reference");
    break;
  case MPU6050_CLOCK_PLL_ZGYRO:
    printfln("PLL with Z axis gyroscope reference");
    break;
  case MPU6050_CLOCK_PLL_YGYRO:
    printfln("PLL with Y axis gyroscope reference");
    break;
  case MPU6050_CLOCK_PLL_XGYRO:
    printfln("PLL with X axis gyroscope reference");
    break;
  case MPU6050_CLOCK_INTERNAL:
    printfln("Internal 8MHz oscillator");
    break;
  }

  printf(" * Accelerometer:             ");
  switch (mpu.getFullScaleAccelRange()) {
  case MPU6050_ACCEL_FS_16:
    printfln("+/- 16 g");
    break;
  case MPU6050_ACCEL_FS_8:
    printfln("+/- 8 g");
    break;
  case MPU6050_ACCEL_FS_4:
    printfln("+/- 4 g");
    break;
  case MPU6050_ACCEL_FS_2:
    printfln("+/- 2 g");
    break;
  }

  printf(" * Accelerometer offsets:     ");
  printf("%d", mpu.getXAccelOffset());
  printf(" / ");
  printf("%d", mpu.getYAccelOffset());
  printf(" / ");
  printfln("%d", mpu.getZAccelOffset());
}

void task_display(void *) {
  mpu = MPU6050();
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

  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);

  mpu.setDHPFMode(MPU6050_DHPF_0P63);

  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);

  mpu.setZeroMotionDetectionThreshold(40);
  mpu.setZeroMotionDetectionDuration(200);

  checkSettings();

  while (1) {
    if (mpu.GetCurrentFIFOPacket(fifoBuffer, packetSize)) {
      mpu.dmpGetAccel(accel, fifoBuffer);
      mpu.dmpGetGravity(grav, fifoBuffer);

      mpu.getAcceleration(&accel_raw[0], &accel_raw[1], &accel_raw[2]);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // mpu.setZeroMotionDetectionThreshold(5);
      printf("%d   ypr: %d, ", mpu.getFIFOCount(), (int)(ypr[0] * 180 / M_PI));
      printf("%d, ", (int)(ypr[1] * 180 / M_PI));
      printf("%d --  %.5d\t%.5d\t%.5d\t\n", (int)(ypr[2] * 180 / M_PI), grav[0], grav[1], grav[2]);
    }
    // printf("%d - %d %d %d\n", mpu.getZeroMotionDetected(), mpu.getXNegMotionDetected(), mpu.getYNegMotionDetected(),
    //        mpu.getZNegMotionDetected());
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}
