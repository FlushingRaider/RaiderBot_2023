/*
  IO_Sensors.hpp

  Created on: Feb 17, 2022
  Author: Biggs

  Contains the code related to the reading and processing of the gyro output.
 */

extern TsRobotSensor VsRobotSensors;

void IO_SensorsInit();

void Read_IO_Sensors(bool L_IR_SensorDetect,
                     bool L_BallSensorLower,
                     bool L_XD_LimitSwitch,
                     bool L_XY_LimitSwitch,
                     bool L_TurretLimitDetected);