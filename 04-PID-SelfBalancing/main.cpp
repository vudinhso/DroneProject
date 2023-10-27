#include <Arduino.h>
#include "MyMPU.h"
#include "MySerial.h"
#include "MyMotorConfig.h"
#include <PID_v1.h>

// ================================================================
// Variable declaration
// ================================================================
unsigned long time_prev = 0;

// PID
// input = sensor; output = pid output; setpoint = reference setpoint
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Innerloop
double gyroX_setpoint = 0, motor_cmd = 0, motor_cmd_tmp = 0;
double kp1 = 2., ki1 = 0.0, kd1 = .0;
PID myPID1(&gyroX, &motor_cmd, &gyroX_setpoint, kp1, ki1, kd1, DIRECT);

// Outerloop
double angleX_setpoint = 0 ;
// double kp2 = 6.0, ki2 = 40.0, kd2 = .0;
double kp2 = 2., ki2 = 0.0, kd2 = .0;
PID myPID2(&angleX, &gyroX_setpoint, &angleX_setpoint, kp2, ki2, kd2, DIRECT);

// //Single Loop
// double angleX_setpoint = 0, motor_cmd = 0, motor_cmd_tmp = 0;
// double kp = 4.0, ki = 2.0, kd = 0.0;
// PID myPID(&angleX, &motor_cmd, &angleX_setpoint, kp, ki, kd, DIRECT);

// ================================================================
// Function Declaration
// ================================================================
void SerialDataPrint();
void SerialDataWrite();

// ================================================================
// Setup function
// ================================================================
void setup()
{
  init_serial();
  init_MPU();
  init_MotorPin();

  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-127, 127);
  // myPID1.SetSampleTime(10);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-127, 127);
  // myPID2.SetSampleTime(10);

  // myPID.SetMode(AUTOMATIC);
  // myPID.SetOutputLimits(-127, 127);
  // myPID.SetSampleTime(10);
}

// ================================================================
// Loop function
// ================================================================
void loop()
{
  getData_MPU();
  // myPID.SetTunings(kp, ki, kd); 
  myPID2.SetTunings(kp2, ki2, kd2); 
  myPID1.SetTunings(kp1, ki1, kd1); 

  myPID2.Compute(); // Outerloop
  myPID1.Compute(); // Innerloop
  // myPID.Compute(); // SingleLoop
  // if (abs(motor_cmd) < 5) motor_cmd = 0; // Dead band
  // if (abs(angleX) < 3 ) motor_cmd = 0; // Dead band
  if (abs(angleX) > 30) motor_cmd = 0; // motor stop when fall
  // if  (motor_cmd > 60)  angleX_setpoint += .05; // auto-leveling
  // if  (motor_cmd <-60)  angleX_setpoint -= .05; // auto-leveling

  motor_cmd_tmp = map(motor_cmd, -127, 127, 0, 256);
  // if (motor_cmd > 0) motor_cmd_tmp = map(motor_cmd, 0, 127, 127+10, 256);
  // if (motor_cmd < 0) motor_cmd_tmp = map(motor_cmd, -127, 0, 0, 127-10);

  ledcWrite(PWM_CHA_AIN1, motor_cmd_tmp);
  ledcWrite(PWM_CHA_AIN2, motor_cmd_tmp);
  ledcWrite(PWM_CHA_CIN1, motor_cmd_tmp);
  ledcWrite(PWM_CHA_CIN2, motor_cmd_tmp);
  SerialDataPrint();
  SerialDataWrite();
}

// ================================================================
// Function Definition
// ================================================================
void SerialDataPrint()
{
  if (micros() - time_prev >= 50000)
  {
    time_prev = micros();
    // Serial.print(millis());
    // Serial.print("\t");
    Serial.print(angleX);
    Serial.print("\t");
    Serial.print(motor_cmd);
    Serial.print("\t");
    Serial.print( kp2);
    Serial.print("\t");
    Serial.print( ki2);
    Serial.print("\t");
    Serial.print( kd2);
    Serial.print("\t");
    Serial.print( angleX_setpoint);
    // Serial.print("\t");
    // Serial.print(gyroX);
    // Serial.print("\t");
    // Serial.print(gyroY);
    // Serial.print("\t");
    // Serial.print(gyroZ);
    Serial.println();
  }
}

// ================================================================
void SerialDataWrite()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      switch (received_chars[0])
      {
      case 'p':
        received_chars.remove(0,1); 
        kp2 = received_chars.toFloat();
        break;
      case 'i':
        received_chars.remove(0,1); 
        ki2 = received_chars.toFloat();
        break;
      case 'd':
        received_chars.remove(0,1); 
        kd2 = received_chars.toFloat();
        break;
      case 'r':
        received_chars.remove(0,1); 
        angleX_setpoint = received_chars.toFloat();
      default:
        break;
      }
      // angleX_setpoint = received_chars.toFloat();
      // if (received_chars[0] == 'p')
      // {
      //   received_chars.remove(0,1); 
      //   kp = received_chars.toFloat();
      // }
      // if (received_chars[0] == 'i')
      // {
      //   received_chars.remove(0,1); 
      //   ki = received_chars.toFloat();
      // }
      received_chars = "";
    }
  }
}
