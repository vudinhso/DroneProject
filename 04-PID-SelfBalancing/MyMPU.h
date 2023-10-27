#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
// Variable declaration
// ================================================================
MPU6050 mpu;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

uint16_t packetSize;    // DMP packet size. Default is 42 bytes.
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;                  // [w, x, y, z]         quaternion container
VectorFloat gravity;           // [x, y, z]            gravity vector
float ypr[3];                  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double angleX, angleY, angleZ; // x, y, z angular position
double gyroX, gyroY, gyroZ;    // x, y, z angular velocity

// ================================================================
// Function Definition
// ================================================================
void readFifoBuffer()
{
  // Clear buffer
  mpu.resetFIFO();
  // Get FIFO count
  fifoCount = mpu.getFIFOCount();
  // Wait for the FIFO to be filled with the correct data number
  while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  angleX = ypr[2] * 180 / M_PI;
  angleY = -ypr[1] * 180 / M_PI;
  angleZ = -ypr[0] * 180 / M_PI;
}
// ================================================================
void init_MPU()
{
  // Wire.begin(I2C_SDA, I2C_SCL);
  Wire.begin(4, 16);
  Wire.setClock(400000);

  mpu.initialize();
  mpu.dmpInitialize();

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();

  // mpu.setDLPFMode(0x02); // MPU6050_DLPF_BW_98
}
// ================================================================
void getData_MPU()
{
  readFifoBuffer();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}
