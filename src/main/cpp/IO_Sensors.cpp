/*
  IO_Sensors.cpp

   Created on: Feb 17, 2022
   Author: Biggs

   Contains the code related to the reading and processing of the IO sensors:
   - IR ball detection
   - XY limit detection
   - XD limit detection

 */

#include "Enums.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

TsRobotSensor VsRobotSensors;  // Structure of all the processed robot sensor signals

/******************************************************************************
 * Function:     IO_SensorsInit
 *
 * Description:  Init calling funciton for the IO sensors.
 *
 ******************************************************************************/
void IO_SensorsInit()
  {
  VsRobotSensors.b_BallDetectedUpper = false;
  VsRobotSensors.b_BallDetectedLower = false;
  VsRobotSensors.b_XD_LimitDetected = false;
  VsRobotSensors.b_XY_LimitDetected = false;
  VsRobotSensors.b_TurretZero = false;
  }

/******************************************************************************
 * Function:     BallDetectionSensor
 *
 * Description:  IR sensor that detects the presence of a ball in the elevator.
 *
 ******************************************************************************/
void BallDetectionSensor(bool L_IR_SensorDetect,
                         bool L_BallSensorLower)
  {
    bool L_BallDetected = false;
    bool L_BallDetectedLower = false;

    if (L_IR_SensorDetect == false)
      {
      L_BallDetected = true;
      }

    if (L_BallSensorLower == false)
      {
      L_BallDetectedLower = true;
      }
    
    VsRobotSensors.b_BallDetectedUpper = L_BallDetected;

    VsRobotSensors.b_BallDetectedLower = L_BallDetectedLower;
  }



/******************************************************************************
 * Function:     ReadLimitSwitchs
 *
 * Description:  Limit switches for the following indications:
 *               - end of travel for XD/YD lifts
 *               - end of travel for turret
 *
 ******************************************************************************/
void ReadLimitSwitchs(bool L_XD_LimitSwitch,
                      bool L_YD_LimitSwitch,
                      bool L_TurretLimitDetected)
  {
    VsRobotSensors.b_XD_LimitDetected = L_XD_LimitSwitch;
    VsRobotSensors.b_XY_LimitDetected = L_YD_LimitSwitch;
    VsRobotSensors.b_TurretZero = !L_TurretLimitDetected; // Invert switch
    frc::SmartDashboard::PutBoolean("XD_work_pls", VsRobotSensors.b_XD_LimitDetected);
  }


/******************************************************************************
 * Function:     Read_IO_Sensors
 *
 * Description:  Main calling funciton for the IO sensors.
 *
 ******************************************************************************/
void Read_IO_Sensors(bool L_IR_SensorDetect,
                     bool L_BallSensorLower,
                     bool L_XD_LimitSwitch,
                     bool L_XY_LimitSwitch,
                     bool L_TurretLimitDetected)
  {
    BallDetectionSensor(L_IR_SensorDetect,
                        L_BallSensorLower);

    ReadLimitSwitchs(L_XD_LimitSwitch,
                     L_XY_LimitSwitch,
                     L_TurretLimitDetected);
  }
