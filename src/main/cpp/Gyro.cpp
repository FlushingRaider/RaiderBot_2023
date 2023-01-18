/*
  Gyro.cpp

   Created on: Feb 01, 2020
   Author: 5561 and CHLOE

   Contains the code related to the reading and processing of the gyro output.

 */

#include "Const.hpp"
#include "ctre/Phoenix.h"

using namespace frc;

WPI_PigeonIMU                              _pidgey{KeGRY_i_Gyro};

double VeGRY_Deg_GyroYawAngleDegrees; //Raw gyro angle in degrees uwu
double VeGRY_Deg_GyroYawAngleDegreesPrev; //Saves previous gyro degree values
double VeGRY_Deg_GyroYawAngleDegreesRate; //
double VeGRY_Deg_GyroYawAngleDegreesRatePrev; //Saves previous gyro rate values
double VeGRY_Rad_GyroYawAngleRad; //Raw gyro angle in radians owo

/******************************************************************************
 * Function:     GyroInit
 *
 * Description:  Initialization of the gyro.
 ******************************************************************************/
void GyroInit()
  {
    _pidgey.ConfigFactoryDefault();
    _pidgey.SetYaw(0.0, KeGRY_ms_GyroTimeoutMs);

    VeGRY_Deg_GyroYawAngleDegrees = 0;
    VeGRY_Deg_GyroYawAngleDegreesPrev = 0;
    VeGRY_Rad_GyroYawAngleRad = 0;
    VeGRY_Deg_GyroYawAngleDegreesRate = 0;
    VeGRY_Deg_GyroYawAngleDegreesRatePrev = 0;
  }

/******************************************************************************
 * Function:     ReadGyro2
 *
 * Description:  Contains the code to read the Pidgeon gyro.
 ******************************************************************************/
void ReadGyro2(bool LeGRY_b_Cmd_DriverZeroGyroCmnd)
  {
  bool Le_GyroValidity = false;//potentially not used elsewhere???
  double LeGRY_Deg_GyroYawAngleRawDegrees    = 0;
  double LeGRY_Deg_GyroYawAngleLimited   = 0;
  double LeGRY_Deg_GyroYawAngularRateDegrees = 0;
  double LaGRY_Rad_GyroRawData[3]; //Rotation in Radians


  if (LeGRY_b_Cmd_DriverZeroGyroCmnd)
    {
    _pidgey.SetYaw(0.0, KeGRY_ms_GyroTimeoutMs);
    }

  LeGRY_Deg_GyroYawAngleRawDegrees    = -_pidgey.GetYaw();
  LeGRY_Deg_GyroYawAngularRateDegrees = LaGRY_Rad_GyroRawData[2];

  LeGRY_Deg_GyroYawAngleLimited  = std::fmod((LeGRY_Deg_GyroYawAngleRawDegrees), 360);

  if (LeGRY_Deg_GyroYawAngleLimited > 180)
    {
    LeGRY_Deg_GyroYawAngleLimited -= 360;
    }
  else if (LeGRY_Deg_GyroYawAngleLimited < -180)
    {
    LeGRY_Deg_GyroYawAngleLimited += 360;
    }


  VeGRY_Deg_GyroYawAngleDegreesPrev     = VeGRY_Deg_GyroYawAngleDegrees;  // Save previous for next loop
  VeGRY_Deg_GyroYawAngleDegrees         = LeGRY_Deg_GyroYawAngleLimited;

  VeGRY_Deg_GyroYawAngleDegreesRatePrev = VeGRY_Deg_GyroYawAngleDegreesRate;  // Save previous for next loop
  VeGRY_Deg_GyroYawAngleDegreesRate     = LeGRY_Deg_GyroYawAngularRateDegrees;

  VeGRY_Rad_GyroYawAngleRad             = LeGRY_Deg_GyroYawAngleLimited / C_RadtoDeg;
  }