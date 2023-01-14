/*
  Gyro.Hpp

   Created on: Feb 01, 2020
   Author: 5561

   Contains the code related to the reading and processing of the gyro output.

 */

extern double V_GyroYawAngleDegrees;
extern double V_GyroYawAngleRad;

void GyroInit();
void ReadGyro2(bool L_DriverZeroGyroCmnd);