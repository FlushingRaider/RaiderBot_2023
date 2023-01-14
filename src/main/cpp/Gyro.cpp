/*
  Gyro.cpp

   Created on: Feb 01, 2020
   Author: 5561

   Contains the code related to the reading and processing of the gyro output.

 */

#include "Const.hpp"
#include "ctre/Phoenix.h"

using namespace frc;

WPI_PigeonIMU                              _pidgey{C_i_Gyro};

double V_GyroYawAngleDegrees;
double V_GyroYawAngleDegreesPrev;
double V_GyroYawAngleDegreesRate;
double V_GyroYawAngleDegreesRatePrev;
double V_GyroYawAngleRad;

/******************************************************************************
 * Function:     GyroInit
 *
 * Description:  Initialization of the gyro.
 ******************************************************************************/
void GyroInit()
  {
    _pidgey.ConfigFactoryDefault();
    _pidgey.SetYaw(0.0, K_t_GyroTimeoutMs);

    V_GyroYawAngleDegrees = 0;
    V_GyroYawAngleDegreesPrev = 0;
    V_GyroYawAngleRad = 0;
    V_GyroYawAngleDegreesRate = 0;
    V_GyroYawAngleDegreesRatePrev = 0;
  }

/******************************************************************************
 * Function:     ReadGyro2
 *
 * Description:  Contains the code to read the Pidgeon gyro.
 ******************************************************************************/
void ReadGyro2(bool L_DriverZeroGyroCmnd)
  {
  bool Le_GyroValidity = false;
  double Le_GyroYawAngleRawDegrees    = 0;
  double Le_Deg_GyroYawAngleLimited   = 0;
  double Le_GyroYawAngularRateDegrees = 0;
  double La_GyroRawData[3];


  if (L_DriverZeroGyroCmnd)
    {
    _pidgey.SetYaw(0.0, K_t_GyroTimeoutMs);
    }

  Le_GyroYawAngleRawDegrees    = -_pidgey.GetYaw();
  Le_GyroYawAngularRateDegrees = La_GyroRawData[2];

  Le_Deg_GyroYawAngleLimited  = std::fmod((Le_GyroYawAngleRawDegrees), 360);

  if (Le_Deg_GyroYawAngleLimited > 180)
    {
    Le_Deg_GyroYawAngleLimited -= 360;
    }
  else if (Le_Deg_GyroYawAngleLimited < -180)
    {
    Le_Deg_GyroYawAngleLimited += 360;
    }


  V_GyroYawAngleDegreesPrev     = V_GyroYawAngleDegrees;  // Save previous for next loop
  V_GyroYawAngleDegrees         = Le_Deg_GyroYawAngleLimited;

  V_GyroYawAngleDegreesRatePrev = V_GyroYawAngleDegreesRate;  // Save previous for next loop
  V_GyroYawAngleDegreesRate     = Le_GyroYawAngularRateDegrees;

  V_GyroYawAngleRad             = Le_Deg_GyroYawAngleLimited / C_RadtoDeg;
  }