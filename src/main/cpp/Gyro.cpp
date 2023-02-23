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
double VeGRY_Rad_GyroYawAngleRad; //Raw gyro angle in radians owo
double VeGRY_Deg_GyroPitchAngleDegrees; //Raw gyro angle in degrees for pitch
double VeGRY_Deg_GyroRollAngleDegrees; 

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
    VeGRY_Rad_GyroYawAngleRad = 0;
    VeGRY_Deg_GyroPitchAngleDegrees = 0;
    VeGRY_Deg_GyroRollAngleDegrees = 0;
  }

/******************************************************************************
 * Function:     ReadGyro2
 *
 * Description:  Contains the code to read the Pidgeon gyro.
 ******************************************************************************/
void ReadGyro2(bool LeGRY_b_Cmd_DriverZeroGyroCmnd)
  {
  double LeGRY_Deg_GyroYawAngleRawDegrees = 0;
  double LeGRY_Deg_GyroYawAngleLimited   = 0;
  double LeGRY_Deg_GyroPitchAngleDegrees = 0;
  double LeGRY_Deg_GyroRollAngleDegrees = 0;
  // double LeGRY_Deg_GyroPitchAngleLimited = 0;

  if (LeGRY_b_Cmd_DriverZeroGyroCmnd)
    {
    _pidgey.SetYaw(0.0, KeGRY_ms_GyroTimeoutMs);
    }

  LeGRY_Deg_GyroYawAngleRawDegrees = -_pidgey.GetYaw();
  LeGRY_Deg_GyroPitchAngleDegrees = _pidgey.GetPitch();
  LeGRY_Deg_GyroRollAngleDegrees = _pidgey.GetRoll();

  LeGRY_Deg_GyroYawAngleLimited  = std::fmod((LeGRY_Deg_GyroYawAngleRawDegrees), 360);

  if (LeGRY_Deg_GyroYawAngleLimited > 180)
    {
    LeGRY_Deg_GyroYawAngleLimited -= 360;
    }
  else if (LeGRY_Deg_GyroYawAngleLimited < -180)
    {
    LeGRY_Deg_GyroYawAngleLimited += 360;
    }

  VeGRY_Deg_GyroYawAngleDegrees = LeGRY_Deg_GyroYawAngleLimited;

  VeGRY_Rad_GyroYawAngleRad = LeGRY_Deg_GyroYawAngleLimited / C_RadtoDeg;

  VeGRY_Deg_GyroPitchAngleDegrees = LeGRY_Deg_GyroPitchAngleDegrees;
  VeGRY_Deg_GyroRollAngleDegrees = LeGRY_Deg_GyroRollAngleDegrees;
  }