/*
  IO_Sensors.cpp

   Created on: Feb 17, 2022
   Revised on: Oct 31, 2023

  Author: Biggs
   Revised by: Chris

   Contains the code related to the reading and processing of the IO sensors:
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
  VsRobotSensors.b_XD_LimitDetected = false;
  VsRobotSensors.b_XY_LimitDetected = false;
  }

/******************************************************************************
 * Function:     ReadLimitSwitchs
 *
 * Description:  Limit switches for the following indications:
 *               - end of travel for XD/YD Manipulators
 *
 ******************************************************************************/
void ReadLimitSwitchs(bool L_XD_LimitSwitch,
                      bool L_YD_LimitSwitch)
  {
    VsRobotSensors.b_XD_LimitDetected = L_XD_LimitSwitch;
    VsRobotSensors.b_XY_LimitDetected = L_YD_LimitSwitch;
  }


/******************************************************************************
 * Function:     Read_IO_Sensors
 *
 * Description:  Main calling funciton for the IO sensors.
 *
 ******************************************************************************/
void Read_IO_Sensors(bool L_XD_LimitSwitch,
                     bool L_XY_LimitSwitch)
  {
    ReadLimitSwitchs(L_XD_LimitSwitch,
                     L_XY_LimitSwitch);
  }
