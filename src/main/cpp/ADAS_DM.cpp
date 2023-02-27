/*
  ADAS_DM.cpp

  Created on: Mar 02, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems) Drive Manuvering (DM)
  Contains the logic and code used for the drive manuvering control.
  As of 03-02, this will just blindly launch the ball and drive forward 
  for a set amount of time.

  Changes:
  2022-03-02 -> Beta, not tested....
 */

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"

double                    V_ADAS_DM_DebounceTime      = 0;
bool                      V_ADAS_DM_StateInit = false;
double                    V_ADAS_DM_Rotate180TargetAngle = 0;
double                    V_ADAS_DM_InitGyroAngle = 0;
double                    V_ADAS_DM_StateTimer = 0;
double                    V_ADAS_DM_X_ErrorPrev = 0;
double                    V_ADAS_DM_Y_ErrorPrev = 0;
double                    V_ADAS_DM_X_Integral = 0;
double                    V_ADAS_DM_Y_Integral = 0;
double                    V_ADAS_DM_X_StartPosition = 0;
double                    V_ADAS_DM_Y_StartPosition = 0;
double                    V_ADAS_DM_X_TargetStartPosition = 0;
double                    V_ADAS_DM_Y_TargetStartPosition = 0;
double                    V_ADAS_DM_TargetAngle;
double                    V_ADAS_DM_GyroPrevious;
bool                      V_ADAS_DM_GyroFlipNeg;
bool                      V_ADAS_DM_GyroFlipPos;
double                    V_ADAS_DM_InitAngle;
double                    V_ADAS_DM_StartAngle;

bool                      VeADAS_b_DM_AutoBalanceInit = false;
bool                      VeADAS_b_DM_AutoBalancePositive = false;
bool                      VeADAS_b_DM_AutoBalanceFastSearch = false;
double                    VeADAS_t_DM_AutoBalanceDbTm = 0.0;
double                    VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
double                    VeADAS_Deg_DM_AutoBalanceErrorPrev = 0;
double                    VeADAS_Deg_DM_AutoBalanceIntegral = 0;

double                    VeADAS_t_DM_DebounceTime = 0;

double V_TargetAngle;
double V_GyroPrevious;
bool V_GyroFlipNeg;
bool V_GyroFlipPos;
double V_OffsettedGyro;



/* Configuration cals: */
// double KV_ADAS_DM_DebounceTime;
// double KV_ADAS_DM_RotateDeadbandAngle;


/******************************************************************************
 * Function:     ADAS_DM_ConfigsInit
 *
 * Description:  Contains the configurations for ADAS DM.
 ******************************************************************************/
void ADAS_DM_ConfigsInit()
  {
    
  // set coefficients
  // KV_ADAS_UT_LightDelayTime = K_ADAS_UT_LightDelayTIme;
  // KV_ADAS_UT_LostTargetGx = K_ADAS_UT_LostTargetGx;
  // KV_ADAS_UT_NoTargetError = K_ADAS_UT_NoTargetError;
  // KV_ADAS_UT_DebounceTime = K_ADAS_UT_DebounceTime;
  // KV_ADAS_UT_AllowedLauncherError = K_ADAS_UT_AllowedLauncherError;
  // KV_ADAS_UT_AllowedLauncherTime = K_ADAS_UT_AllowedLauncherTime;
  // KV_ADAS_UT_RotateDeadbandAngle = K_ADAS_UT_RotateDeadbandAngle;
  // KV_ADAS_UT_TargetVisionAngle = K_ADAS_UT_TargetVisionAngle;

  // #ifdef ADAS_DM_Test
  // // display coefficients on SmartDashboard
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_LightDelayTime", KV_ADAS_UT_LightDelayTime);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_LostTargetGx", KV_ADAS_UT_LostTargetGx);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_NoTargetError", KV_ADAS_UT_NoTargetError);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_DebounceTime", KV_ADAS_UT_DebounceTime);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_AllowedLauncherError", KV_ADAS_UT_AllowedLauncherError);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_AllowedLauncherTime", KV_ADAS_UT_AllowedLauncherTime);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_RotateDeadbandAngle", KV_ADAS_UT_RotateDeadbandAngle);
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_TargetVisionAngle", KV_ADAS_UT_TargetVisionAngle);
  // #endif
  }


/******************************************************************************
 * Function:     ADAS_DM_ConfigsCal
 *
 * Description:  Contains the configurations for ADAS DM.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ADAS_DM_ConfigsCal()
  {
  // read coefficients from SmartDashboard
  #ifdef ADAS_DM_Test
  // KV_ADAS_UT_LightDelayTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LightDelayTime", KV_ADAS_UT_LightDelayTime);
  // KV_ADAS_UT_LostTargetGx = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LostTargetGx", KV_ADAS_UT_LostTargetGx);
  // KV_ADAS_UT_NoTargetError = frc::SmartDashboard::GetNumber("KV_ADAS_UT_NoTargetError", KV_ADAS_UT_NoTargetError);
  // KV_ADAS_UT_DebounceTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_DebounceTime", KV_ADAS_UT_DebounceTime);
  // KV_ADAS_UT_AllowedLauncherError = frc::SmartDashboard::GetNumber("KV_ADAS_UT_AllowedLauncherError", KV_ADAS_UT_AllowedLauncherError);
  // KV_ADAS_UT_AllowedLauncherTime = frc::SmartDashboard::GetNumber("KV_ADAS_UT_AllowedLauncherTime", KV_ADAS_UT_AllowedLauncherTime);
  // KV_ADAS_UT_RotateDeadbandAngle = frc::SmartDashboard::GetNumber("KV_ADAS_UT_RotateDeadbandAngle", KV_ADAS_UT_RotateDeadbandAngle);
  // KV_ADAS_UT_TargetVisionAngle = frc::SmartDashboard::GetNumber("KV_ADAS_UT_TargetVisionAngle", KV_ADAS_UT_TargetVisionAngle);
  #endif
  }


/******************************************************************************
 * Function:     ADAS_DM_Reset
 *
 * Description:  Reset all applicable DM variables.
 ******************************************************************************/
void ADAS_DM_Reset(void)
  {
  V_ADAS_DM_DebounceTime      = 0;
  V_ADAS_DM_StateInit = false;
  V_ADAS_DM_Rotate180TargetAngle = 0;
  V_ADAS_DM_InitGyroAngle = 0;
  V_ADAS_DM_StateTimer = 0;
  V_ADAS_DM_X_ErrorPrev = 0;
  V_ADAS_DM_Y_ErrorPrev = 0;
  V_ADAS_DM_X_Integral = 0;
  V_ADAS_DM_Y_Integral = 0;
  V_ADAS_DM_X_StartPosition = 0;
  V_ADAS_DM_Y_StartPosition = 0;
  V_ADAS_DM_X_TargetStartPosition = 0;
  V_ADAS_DM_Y_TargetStartPosition = 0;
  V_ADAS_DM_InitAngle = 0;
  VeADAS_b_DM_AutoBalanceInit = false;
  VeADAS_b_DM_AutoBalancePositive = false;
  VeADAS_b_DM_AutoBalanceFastSearch = false;
  VeADAS_t_DM_AutoBalanceDbTm = 0.0;
  VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
  VeADAS_Deg_DM_AutoBalanceErrorPrev = 0;
  VeADAS_Deg_DM_AutoBalanceIntegral = 0;
  }


/******************************************************************************
 * Function:     ADAS_DM_Rotate180
 *
 * Description:  Rotate 180 degrees control.
 ******************************************************************************/
bool ADAS_DM_Rotate180(double     *L_Pct_FwdRev,
                       double     *L_Pct_Strafe,
                       double     *L_Pct_Rotate,
                       double     *L_RPM_Launcher,
                       double     *L_Pct_Intake,
                       double     *L_Pct_Elevator,
                       bool       *L_CameraUpperLightCmndOn,
                       bool       *L_CameraLowerLightCmndOn,
                       bool       *L_SD_RobotOriented,
                       double      L_Deg_GyroAngleDeg)
  {
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;

  *L_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  if (V_ADAS_DM_StateInit == false)
    {
    /* Need to find the target angle.  The gyro in use will only report values of -180 to 180. Need to account for this:*/
    V_ADAS_DM_Rotate180TargetAngle = L_Deg_GyroAngleDeg - 180;

    // V_ADAS_DM_Rotate180TargetAngle = std::fmod((V_ADAS_DM_Rotate180TargetAngle), 180);

    if (V_ADAS_DM_Rotate180TargetAngle >= 180)
      {
      V_ADAS_DM_Rotate180TargetAngle -= 360;
      }
    else if (V_ADAS_DM_Rotate180TargetAngle <= -180)
      {
      V_ADAS_DM_Rotate180TargetAngle += 360;
      }

    V_ADAS_DM_StateInit = true;
    }

  L_RotateError = V_ADAS_DM_Rotate180TargetAngle - L_Deg_GyroAngleDeg;

  if (fabs(L_RotateError) <= K_ADAS_DM_RotateDeadbandAngle && V_ADAS_DM_DebounceTime < K_ADAS_DM_RotateDebounceTime)
    {
    V_ADAS_DM_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_RotateError) > K_ADAS_DM_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_DM_DebounceTime = 0;
    }
  else if (V_ADAS_DM_DebounceTime >= K_ADAS_DM_RotateDebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_DebounceTime = 0;
    }

  if (L_ADAS_DM_StateComplete == false)
    {
    *L_Pct_Rotate = DesiredRotateSpeed(L_RotateError);
    }
  else
    {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *L_Pct_Rotate = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_StateInit = false;
    }

  return (L_ADAS_DM_StateComplete);
  }
/******************************************************************************
 * Function:     ADAS_DM_RotateTo
 *
 * Description:  Rotate to a given position (L_Deg_GyroAngleTarget)
 ******************************************************************************/
bool ADAS_DM_RotateTo(double     *L_Pct_FwdRev,
                       double     *L_Pct_Strafe,
                       double     *L_Pct_Rotate,
                       double     *L_RPM_Launcher,
                       double     *L_Pct_Intake,
                       double     *L_Pct_Elevator,
                       bool       *L_CameraUpperLightCmndOn,
                       bool       *L_CameraLowerLightCmndOn,
                       bool       *L_SD_RobotOriented,
                       double      L_Deg_GyroAngleDeg,
                       double      L_Deg_GyroAngleTarget)
  {
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;
  double L_ClockwiseError = 0;
  double L_CounterClockwiseError = 0;

  *L_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  if (V_ADAS_DM_StateInit == false)
    {
    /* Need to find the target angle.  The gyro in use will only report values of -180 to 180. Need to account for this:*/
    //V_ADAS_DM_Rotate180TargetAngle = L_InitGyroAngle - 180;

    // V_ADAS_DM_Rotate180TargetAngle = std::fmod((V_ADAS_DM_Rotate180TargetAngle), 180);

    L_ClockwiseError = fabs(L_Deg_GyroAngleTarget - L_Deg_GyroAngleDeg);

    L_CounterClockwiseError = fabs((L_Deg_GyroAngleTarget - 360) - L_Deg_GyroAngleDeg);

    if (L_CounterClockwiseError < L_ClockwiseError)
      {
      V_TargetAngle = L_CounterClockwiseError - 360;
      V_GyroPrevious = L_Deg_GyroAngleDeg;
      }

    if ((V_GyroPrevious <= -170 && L_Deg_GyroAngleDeg >= 170) ||
        (V_GyroFlipNeg && L_Deg_GyroAngleDeg < 0))
      {
      V_OffsettedGyro = L_Deg_GyroAngleDeg - 360;
      V_GyroFlipNeg = true;
      }
    else if (V_GyroPrevious >= 170 && L_Deg_GyroAngleDeg <=170)
      {
      V_OffsettedGyro = L_Deg_GyroAngleDeg + 360;
      V_GyroFlipPos = true;
      }
    else
      {
      V_OffsettedGyro = L_Deg_GyroAngleDeg;
    }


    V_GyroPrevious = L_Deg_GyroAngleDeg;

    if (V_ADAS_DM_Rotate180TargetAngle > 180)
      {
      V_ADAS_DM_Rotate180TargetAngle -= 360;
      }
    else if (V_ADAS_DM_Rotate180TargetAngle < -180)
      {
      V_ADAS_DM_Rotate180TargetAngle += 360;
      }

    if (V_ADAS_DM_Rotate180TargetAngle >= -179 || V_ADAS_DM_Rotate180TargetAngle >= 179)
     {
       V_ADAS_DM_Rotate180TargetAngle = 178;
     } 

    V_ADAS_DM_StateInit = true;
    }

  L_RotateError = V_ADAS_DM_Rotate180TargetAngle - L_Deg_GyroAngleDeg;

  if (fabs(L_RotateError) <= K_ADAS_DM_RotateDeadbandAngle && V_ADAS_DM_DebounceTime < K_ADAS_DM_RotateDebounceTime)
    {
    V_ADAS_DM_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_RotateError) > K_ADAS_DM_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_DM_DebounceTime = 0;
    }
  else if (V_ADAS_DM_DebounceTime >= K_ADAS_DM_RotateDebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_DebounceTime = 0;
    }

  if (L_ADAS_DM_StateComplete == false)
    {
    *L_Pct_Rotate = DesiredRotateSpeed(L_RotateError);
    }
  else
    {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *L_Pct_Rotate = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_StateInit = false;
    }

  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_FieldOrientRotate
 *
 * Description:  Rotate to the zeroed position
 ******************************************************************************/
bool ADAS_DM_FieldOrientRotate(double     *L_Pct_FwdRev,
                               double     *L_Pct_Strafe,
                               double     *L_Pct_Rotate,
                               double     *L_Pct_Intake,
                               bool       *L_SD_RobotOriented,
                               double      L_Deg_GyroAngleDeg,
                               double      L_Deg_GyroAngleTarget)
  {
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;
  double L_Deg_TargetErrorActual = 0;
  double L_Deg_GyroRolloverProtected = 0;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Intake = 0;

  if (V_ADAS_DM_StateInit == false)
    {
    /* Need to find the target angle.  The gyro in use will only report values of -180 to 180. Need to account for this:*/
    //V_ADAS_DM_Rotate180TargetAngle = L_InitGyroAngle - 180;

    // V_ADAS_DM_Rotate180TargetAngle = std::fmod((V_ADAS_DM_Rotate180TargetAngle), 180);

    L_RotateError = L_Deg_GyroAngleTarget - L_Deg_GyroAngleDeg;

    /* A positive error number indicates clockwise rotation */
    if (L_RotateError > 180)
      {
      /* It would be quicker to rotate in a counter clockwise direction to meet the command*/
      L_RotateError -= 360;
      }
    else if (L_RotateError < -180)
      {
      L_RotateError += 360;
      }
    
    V_ADAS_DM_TargetAngle = L_RotateError;
    V_ADAS_DM_StateInit = true;
    }

  L_RotateError = V_TargetAngle - L_Deg_GyroAngleDeg;

  /* Detect Gyro roll over. */
  if ((V_ADAS_DM_GyroPrevious <= -170 && L_Deg_GyroAngleDeg >= 170) ||
      (V_ADAS_DM_GyroFlipNeg == true  && L_Deg_GyroAngleDeg > 0))
    {
    L_Deg_GyroRolloverProtected = L_Deg_GyroAngleDeg - 360;
    V_ADAS_DM_GyroFlipNeg = true;
    V_ADAS_DM_GyroFlipPos = false;
    }
  else if ((V_GyroPrevious >= 170 && L_Deg_GyroAngleDeg <=170) || 
           (V_GyroFlipPos && L_Deg_GyroAngleDeg < 0))
    {
    L_Deg_GyroRolloverProtected = L_Deg_GyroAngleDeg + 360;
    V_ADAS_DM_GyroFlipPos = true;
    V_ADAS_DM_GyroFlipNeg = false;
    }
  else
    {
    L_Deg_GyroRolloverProtected = L_Deg_GyroAngleDeg;
    V_ADAS_DM_GyroFlipPos = false;
    V_ADAS_DM_GyroFlipNeg = false;
    }

  L_Deg_TargetErrorActual = V_TargetAngle - L_Deg_GyroRolloverProtected;
  V_ADAS_DM_GyroPrevious = L_Deg_GyroAngleDeg;

  if (fabs(L_Deg_TargetErrorActual) <= K_ADAS_DM_RotateDeadbandAngle && V_ADAS_DM_DebounceTime < K_ADAS_DM_RotateDebounceTime)
    {
    V_ADAS_DM_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_Deg_TargetErrorActual) > K_ADAS_DM_RotateDeadbandAngle)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_DM_DebounceTime = 0;
    }
  else if (V_ADAS_DM_DebounceTime >= K_ADAS_DM_RotateDebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_DebounceTime = 0;
    }

  if (L_ADAS_DM_StateComplete == false)
    {
    *L_Pct_Rotate = DesiredRotateSpeed(L_Deg_TargetErrorActual);
    }
  else
    {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *L_Pct_Rotate = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_StateInit = false;
    V_ADAS_DM_GyroPrevious = 0;
    V_ADAS_DM_GyroFlipPos = false;
    V_ADAS_DM_GyroFlipNeg = false;
    }

  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_DriveStraight
 *
 * Description:  Drive straight control.
 ******************************************************************************/
bool ADAS_DM_DriveStraight(double     *L_Pct_FwdRev,
                           double     *L_Pct_Strafe,
                           double     *L_Pct_Rotate,
                           double     *L_RPM_Launcher,
                           double     *L_Pct_Intake,
                           double     *L_Pct_Elevator,
                           bool       *L_CameraUpperLightCmndOn,
                           bool       *L_CameraLowerLightCmndOn,
                           bool       *L_SD_RobotOriented)
  {
  bool L_ADAS_DM_StateComplete = false;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  V_ADAS_DM_DebounceTime += C_ExeTime; // update our timekeeping

  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_DriveTimeLong) // check that we are still in the time we have given ourselves
    {
    *L_Pct_FwdRev = K_ADAS_DM_DriveFWD_Pct; // set our strafe percent to this constant
    }
  else
    {
    *L_Pct_FwdRev = 0;                      // reset all the variables a driver state
    *L_SD_RobotOriented = false;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    }
  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_ReverseAndIntake
 *
 * Description:  Drive in reverse and turn on intake.
 ******************************************************************************/
bool ADAS_DM_ReverseAndIntake(double     *L_Pct_FwdRev,
                              double     *L_Pct_Strafe,
                              double     *L_Pct_Rotate,
                              double     *L_RPM_Launcher,
                              double     *L_Pct_Intake,
                              double     *L_Pct_Elevator,
                              bool       *L_CameraUpperLightCmndOn,
                              bool       *L_CameraLowerLightCmndOn,
                              bool       *L_SD_RobotOriented,
                              double      L_DriveTime)
  {
  bool L_ADAS_DM_StateComplete = false;

  *L_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = K_ADAS_DM_BlindShotIntake;
  *L_Pct_Elevator = 0; // Elevator should automatically enable when necessary when intake is commanded on

  V_ADAS_DM_DebounceTime += C_ExeTime;

  if (V_ADAS_DM_DebounceTime <= L_DriveTime)
    {
    *L_Pct_FwdRev = K_ADAS_DM_DriveREV_Pct;
    }
  else
    {
    *L_Pct_FwdRev = 0;
    *L_SD_RobotOriented = false;
    *L_Pct_Intake = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    }
  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_BlindShot
 *
 * Description:  Blindly shoot the ball at a prescribed speed.
 ******************************************************************************/
bool ADAS_DM_BlindShot(double       *L_Pct_FwdRev,
                       double       *L_Pct_Strafe,
                       double       *L_Pct_Rotate,
                       double       *L_RPM_Launcher,
                       double       *L_Pct_Intake,
                       double       *L_Pct_Elevator,
                       bool         *L_CameraUpperLightCmndOn,
                       bool         *L_CameraLowerLightCmndOn,
                       bool         *L_SD_RobotOriented)
  {
  bool L_ADAS_DM_StateComplete = false;

  *L_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  *L_Pct_FwdRev = 0;

  V_ADAS_DM_DebounceTime += C_ExeTime;

  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_BlindShotTime)
    {
    *L_Pct_Intake = K_ADAS_DM_BlindShotElevator;
    *L_Pct_Elevator = K_ADAS_DM_BlindShotElevator;
    *L_RPM_Launcher = K_ADAS_DM_BlindShotLauncherHigh;
    }
  else
    {
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    *L_Pct_Intake = 0;
    *L_Pct_Elevator = 0;
    *L_RPM_Launcher = 0;
    }
  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_PathFollower
 *
 * Description:  Follow the preplaned path.  We take in the current field 
 *               position, then lookup the desired position based on the 
 *               current time.  Based on this "error" between the current 
 *               position and the desired position, we command either the 
 *               robot to move foward/backward or to strafe to try and 
 *               reach the desired position
 ******************************************************************************/
bool ADAS_DM_PathFollower(double *L_Pct_FwdRev,
                          double *L_Pct_Strafe,
                          double *L_Pct_Rotate,
                          double *L_Pct_Intake,
                          bool   *L_SD_RobotOriented,
                          double  L_L_X_FieldPos,
                          double  L_L_Y_FieldPos,
                          double  L_Deg_GyroAngleDeg,
                          int     L_i_PathNum,
                          std::string V_ADAS_Auto_PathName)
  {
  bool   L_ADAS_DM_StateComplete = false;
  double L_L_TargetPositionX = 0.0;
  double L_L_TargetPositionY = 0.0;
  double L_Rad_TargetAngle = 0.0;
  double L_L_RelativePosX = 0.0;
  double L_L_RelativePosY = 0.0;
  double L_L_X_Error = 0.0;
  double L_L_Y_Error = 0.0;
  double L_Deg_RotateError = 0.0;
  double L_Deg_RotateTarget = 0.0;
  bool L_timeEND = false;

  /* Set the things we are not using to off: */
  *L_SD_RobotOriented = false;
  *L_Pct_Intake = 0;

  /* Look up the desired target location point: */
  L_timeEND = DesiredAutonLocation2(V_ADAS_DM_StateTimer,
                                    L_i_PathNum,
                                    V_ADAS_Auto_PathName,
                                   &L_L_TargetPositionX,
                                   &L_L_TargetPositionY,
                                   &L_Rad_TargetAngle);

  /* Capture some of the things we need to save for this state control: */
  if (V_ADAS_DM_StateInit == false) {
    V_ADAS_DM_X_StartPosition = L_L_X_FieldPos;
    V_ADAS_DM_Y_StartPosition = L_L_Y_FieldPos;
    V_ADAS_DM_X_TargetStartPosition = L_L_TargetPositionX;
    V_ADAS_DM_Y_TargetStartPosition = L_L_TargetPositionY;
    V_ADAS_DM_InitAngle = L_Deg_GyroAngleDeg;
    V_ADAS_DM_StartAngle = L_Rad_TargetAngle;
    V_ADAS_DM_StateInit = true;
    }

  /* We need to offset the position by the start position since the odometry will 
     start at zero, but the lookup table will not */
  L_L_TargetPositionX -= V_ADAS_DM_X_TargetStartPosition;
  L_L_TargetPositionY -= V_ADAS_DM_Y_TargetStartPosition;

  L_L_RelativePosX = L_L_X_FieldPos - V_ADAS_DM_X_StartPosition;
  L_L_RelativePosY = L_L_Y_FieldPos - V_ADAS_DM_Y_StartPosition;

  L_L_X_Error = fabs(L_L_TargetPositionX - L_L_RelativePosX);
  L_L_Y_Error = fabs(L_L_TargetPositionY - L_L_RelativePosY);

  L_Deg_RotateError = (L_Rad_TargetAngle - V_ADAS_DM_StartAngle - V_ADAS_DM_InitAngle) * C_RadtoDeg - L_Deg_GyroAngleDeg;
  
  L_Deg_RotateTarget = (L_Rad_TargetAngle * C_RadtoDeg);

  V_ADAS_DM_StateTimer += C_ExeTime;

  /* Exit criteria: */
  if (fabs(L_Deg_RotateError) <= K_ADAS_DM_RotateDeadbandAngle && 
      L_L_X_Error <= K_ADAS_DM_XY_Deadband &&
      L_L_Y_Error <= K_ADAS_DM_XY_Deadband &&
      V_ADAS_DM_DebounceTime < K_ADAS_DM_RotateDebounceTime &&
      L_timeEND == true)
    {
    V_ADAS_DM_DebounceTime += C_ExeTime;
    }
  else if (fabs(L_Deg_RotateError) > K_ADAS_DM_RotateDeadbandAngle ||
           L_L_X_Error > K_ADAS_DM_XY_Deadband ||
           L_L_Y_Error > K_ADAS_DM_XY_Deadband)
    {
    /* Reset the timer, we have gone out of bounds */
    V_ADAS_DM_DebounceTime = 0;
    }
  else if (V_ADAS_DM_DebounceTime >= K_ADAS_DM_RotateDebounceTime)
    {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_DebounceTime = 0;
    }

  if (L_ADAS_DM_StateComplete == false)
    {
    *L_Pct_Strafe =  -Control_PID( L_L_TargetPositionX,
                                  L_L_RelativePosX,
                                 &V_ADAS_DM_X_ErrorPrev,
                                 &V_ADAS_DM_X_Integral,
                                  K_k_AutonX_PID_Gx[E_P_Gx],
                                  K_k_AutonX_PID_Gx[E_I_Gx],
                                  K_k_AutonX_PID_Gx[E_D_Gx],
                                  K_k_AutonX_PID_Gx[E_P_Ul],
                                  K_k_AutonX_PID_Gx[E_P_Ll],
                                  K_k_AutonX_PID_Gx[E_I_Ul],
                                  K_k_AutonX_PID_Gx[E_I_Ll],
                                  K_k_AutonX_PID_Gx[E_D_Ul],
                                  K_k_AutonX_PID_Gx[E_D_Ll],
                                  K_k_AutonX_PID_Gx[E_Max_Ul],
                                  K_k_AutonX_PID_Gx[E_Max_Ll]);

     *L_Pct_FwdRev =  -Control_PID( L_L_TargetPositionY,
                                  L_L_RelativePosY,
                                  &V_ADAS_DM_Y_ErrorPrev,
                                  &V_ADAS_DM_Y_Integral,
                                   K_k_AutonY_PID_Gx[E_P_Gx],
                                   K_k_AutonY_PID_Gx[E_I_Gx],
                                   K_k_AutonY_PID_Gx[E_D_Gx],
                                   K_k_AutonY_PID_Gx[E_P_Ul],
                                   K_k_AutonY_PID_Gx[E_P_Ll],
                                   K_k_AutonY_PID_Gx[E_I_Ul],
                                   K_k_AutonY_PID_Gx[E_I_Ll],
                                   K_k_AutonY_PID_Gx[E_D_Ul],
                                   K_k_AutonY_PID_Gx[E_D_Ll],
                                   K_k_AutonY_PID_Gx[E_Max_Ul],
                                   K_k_AutonY_PID_Gx[E_Max_Ll]);
   
     //*L_Pct_Rotate = DesiredRotateSpeed(L_Deg_RotateError);

      // ADAS_DM_FieldOrientRotate(L_Pct_FwdRev,
      //                           L_Pct_Strafe,
      //                           L_Pct_Rotate,
      //                           L_Pct_Intake,
      //                           L_SD_RobotOriented,
      //                           L_Deg_GyroAngleDeg,
      //                           L_Deg_RotateTarget);

    }
  else
    {
    /* We have been at the correct location for the set amount of time. */
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    V_ADAS_DM_DebounceTime = 0;
    L_ADAS_DM_StateComplete = true;
    V_ADAS_DM_StateInit = false;
    V_ADAS_DM_X_StartPosition = 0;
    V_ADAS_DM_Y_StartPosition = 0;
    V_ADAS_DM_X_TargetStartPosition = 0;
    V_ADAS_DM_Y_TargetStartPosition = 0;
    }

  // frc::SmartDashboard::PutBoolean("State Complete", L_ADAS_DM_StateComplete);
  // frc::SmartDashboard::PutNumber("L_Pct_Strafe", *L_Pct_Strafe);
  // frc::SmartDashboard::PutNumber("L_Pct_FwdRev", *L_Pct_FwdRev);
  // frc::SmartDashboard::PutNumber("L_i_PathNum", L_i_PathNum);

  // frc::SmartDashboard::PutNumber("L_L_TargetPositionX", L_L_TargetPositionX);
  // frc::SmartDashboard::PutNumber("L_L_RelativePosX", L_L_RelativePosX);

  // frc::SmartDashboard::PutNumber("L_L_TargetPositionY", L_L_TargetPositionY);
  // frc::SmartDashboard::PutNumber("L_L_RelativePosY", L_L_RelativePosY);

  return (L_ADAS_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_AutoBalance
 *
 * Description:  Balance on the charge station
 ******************************************************************************/
bool ADAS_DM_AutoBalance(double *L_Pct_FwdRev,
                          double *L_Pct_Strafe,
                          double *L_Pct_Rotate,
                          bool   *L_SD_RobotOriented,
                          bool   *LeADAS_b_X_Mode,
                          double  LeADAS_Deg_GyroRoll)
  {
  bool   LeADAS_b_DM_StateComplete = false;
  double LeADAS_Deg_RollError = 0;
  T_PID_Cal L_Index = E_P_Gx;
  double LeADAS_k_DM_AutoBalancePID[E_PID_CalSz];

  /* Look up the desired target location point: */


  /* Capture some of the things we need to save for this state control: */
  if (VeADAS_b_DM_AutoBalanceInit == false)
    {
      if (LeADAS_Deg_GyroRoll > 0)
        {
          VeADAS_b_DM_AutoBalancePositive = true;
        }
      else
        {
          VeADAS_b_DM_AutoBalancePositive = false;
        }
      VeADAS_b_DM_AutoBalanceFastSearch = true;
    }

  /* Detect roll over here: */
  if (VeADAS_b_DM_AutoBalanceFastSearch == true &&
      (((VeADAS_b_DM_AutoBalancePositive == true)  && (LeADAS_Deg_GyroRoll < 0)) ||
       ((VeADAS_b_DM_AutoBalancePositive == false) && (LeADAS_Deg_GyroRoll > 0))))
    {
      VeADAS_b_DM_AutoBalanceFastSearch = false;
    }

  if (VeADAS_b_DM_AutoBalanceFastSearch == true)
    {
      for (L_Index = E_P_Gx;
           L_Index < E_PID_CalSz;
           L_Index = T_PID_Cal(int(L_Index) + 1))
        {
          LeADAS_k_DM_AutoBalancePID[L_Index] = KeADAS_k_DM_AutoBalanceFastPID[L_Index];
        }
    }
  else
    {
      for (L_Index = E_P_Gx;
           L_Index < E_PID_CalSz;
           L_Index = T_PID_Cal(int(L_Index) + 1))
        {
          LeADAS_k_DM_AutoBalancePID[L_Index] = KeADAS_k_DM_AutoBalanceSlowPID[L_Index];
        }
    }

  LeADAS_Deg_RollError = LeADAS_Deg_GyroRoll;

  /* Exit criteria: */
  if (fabs(LeADAS_Deg_RollError) <= KeADAS_Deg_DM_AutoBalanceDb && 
      VeADAS_t_DM_AutoBalanceDbTm < KeADAS_t_DM_AutoBalanceDb)
    {
    VeADAS_t_DM_AutoBalanceDbTm += C_ExeTime;
    }
  else if (fabs(LeADAS_Deg_RollError) > KeADAS_Deg_DM_AutoBalanceDb)
    {
    /* Reset the timer, we have gone out of bounds */
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    }
  else if (VeADAS_t_DM_AutoBalanceDbTm >= KeADAS_Deg_DM_AutoBalanceDb)
    {
    /* Reset the time, proceed to next state. */
    LeADAS_b_DM_StateComplete = true;
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    VeADAS_t_DM_AutoBalanceHoldTm += C_ExeTime;
    }

  if (LeADAS_b_DM_StateComplete == false)
    {
     *L_Pct_FwdRev =  Control_PID(0.0,
                                  LeADAS_Deg_GyroRoll,
                                  &VeADAS_Deg_DM_AutoBalanceErrorPrev,
                                  &VeADAS_Deg_DM_AutoBalanceIntegral,
                                   LeADAS_k_DM_AutoBalancePID[E_P_Gx],
                                   LeADAS_k_DM_AutoBalancePID[E_I_Gx],
                                   LeADAS_k_DM_AutoBalancePID[E_D_Gx],
                                   LeADAS_k_DM_AutoBalancePID[E_P_Ul],
                                   LeADAS_k_DM_AutoBalancePID[E_P_Ll],
                                   LeADAS_k_DM_AutoBalancePID[E_I_Ul],
                                   LeADAS_k_DM_AutoBalancePID[E_I_Ll],
                                   LeADAS_k_DM_AutoBalancePID[E_D_Ul],
                                   LeADAS_k_DM_AutoBalancePID[E_D_Ll],
                                   LeADAS_k_DM_AutoBalancePID[E_Max_Ul],
                                   LeADAS_k_DM_AutoBalancePID[E_Max_Ll]);
   
     *L_Pct_Strafe = 0;
     *L_Pct_Rotate = 0;
     *L_SD_RobotOriented = true;
     *LeADAS_b_X_Mode = false;
    }
  else if ((VeADAS_t_DM_AutoBalanceHoldTm >= C_ExeTime) &&
           (VeADAS_t_DM_AutoBalanceHoldTm <= KeADAS_t_DM_AutoBalanceHold))
    {
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *L_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = true;
    VeADAS_t_DM_AutoBalanceHoldTm += C_ExeTime;
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    LeADAS_b_DM_StateComplete = false;
    VeADAS_b_DM_AutoBalanceInit = false;
    }
  else
    {
    /* We have been at the correct location for the set amount of time. */
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *L_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = false;
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
    LeADAS_b_DM_StateComplete = true;
    VeADAS_b_DM_AutoBalanceInit = false;
    }

  return (LeADAS_b_DM_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_DM_MoveToTag
 * Author: Carson
 * Description: Moves to the closest tag scanned
 ******************************************************************************/
bool ADAS_DM_MoveToTag(double *L_Pct_FwdRev,
                       double *L_Pct_Strafe,
                       double *L_Pct_Rotate,
                       bool L_OdomCentered,
                       int L_TagID,
                       double L_OdometryX,
                       double L_OdometryY,
                       bool *L_VisionTargetingRequest,
                       double L_VisionTopTargetAquired,
                       double L_TagYawDegrees,
                       frc::DriverStation::Alliance LeLC_e_AllianceColor,
                       bool L_CubeAlignCmd,
                       bool L_ConeAlignCmd)
{

  bool LeADAS_b_DM_StateComplete = false;

  // *L_Pct_FwdRev = 0.0;
  // *L_Pct_Strafe = 0.0;
  /* Next, let's set all the other items we aren't trying to control to off: */
  double L_ChosenX = 0.0;
  double L_ChosenY = 0.0;
  double L_ErrorCalcYaw = 0.0;
  int L_ClosestTag;

  // all 3 tags are directly across from each other

  // coord of tag ID 1 and 8
  double L_Tag1Y = 1.071626;
  // coord of tag ID 2 and 7
  double L_Tag2Y = 2.748026;
  // coord of tag ID 3 and 6
  double L_Tag3Y = 4.424426;
  double L_TagXred = 15.513558;
  double L_TagXblue = 1.02743;

  double L_Tag1YError;
  double L_Tag2YError;
  double L_Tag3YError;

  
  if (L_OdomCentered) // don't do any of this if we haven't centered our Odometry based on the tag
  {
    VeADAS_t_DM_DebounceTime += C_ExeTime;
    // pick the right side of tags to look at for our alliance
    if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
    {
      L_ChosenX = L_TagXred; // all 3 tags on the red alliance have the same X
      if (L_TagID == 1)
      {
        L_ChosenY = L_Tag1Y;
      }
      else if (L_TagID == 2)
      {
        L_ChosenY = L_Tag2Y;
      }
      else if (L_TagID == 3)
      {
        L_ChosenY = L_Tag3Y;
      }
    }
    else
    {
      L_ChosenX = L_TagXblue; // x coord of all blue tags
      if (L_TagID == 8)
      {
        L_ChosenY = L_Tag1Y;
      }
      else if (L_TagID == 7)
      {
        L_ChosenY = L_Tag2Y;
      }
      else if (L_TagID == 6)
      {
        L_ChosenY = L_Tag3Y;
      }
    }

    // find the closest tag and our error to it:

      // // check if Error 1 is the largest
      // if (L_Tag1YError >= L_Tag2YError && L_Tag1YError >= L_Tag3YError)
      // {
      //   L_ChosenY = L_Tag1Y;
      // }
      // // check if Error 2 is the largest number
      // else if (L_Tag2YError >= L_Tag1YError && L_Tag2YError >= L_Tag3YError)
      // {
      //   L_ChosenY = L_Tag2Y;
      // }
      // // if neither n1 nor n2 are the largest, Error 3 is the largest
      // else
      // {
      //   L_ChosenY = L_Tag3Y;
      // }

       L_ErrorCalcYaw = 0.0 - L_TagYawDegrees;
    if (VeADAS_t_DM_DebounceTime <= KeADAS_t_DM_TagCenteringDb) // make sure we're still in the time we've given ourselves
    {
      if (L_ErrorCalcYaw > 0 || L_ErrorCalcYaw < 0)
      {
        *L_Pct_Rotate = DesiredAutoRotateSpeed(L_ErrorCalcYaw);
      }
      if (L_OdometryX < L_ChosenX + K_MoveToTagMovementDeadband || L_OdometryX < L_ChosenX - K_MoveToTagMovementDeadband)
      {
        *L_Pct_FwdRev = 0.2 * (L_OdometryX - L_ChosenX);
      }
      else if (L_OdometryX > L_ChosenX + K_MoveToTagMovementDeadband || L_OdometryX > L_ChosenX - K_MoveToTagMovementDeadband)
      {
        *L_Pct_FwdRev = -0.2 * (L_OdometryX - L_ChosenX);
      }
      else
      {
        *L_Pct_FwdRev = 0.0;
      }
      if (L_OdometryY < L_ChosenY + K_MoveToTagMovementDeadband || L_OdometryY < L_ChosenY - K_MoveToTagMovementDeadband)
      {
        *L_Pct_Strafe = 0.2  * (L_OdometryY - L_ChosenY);
      }
      else if (L_OdometryY > L_ChosenY + K_MoveToTagMovementDeadband || L_OdometryY > L_ChosenY - K_MoveToTagMovementDeadband)
      {
        *L_Pct_Strafe = -0.2  * (L_OdometryY - L_ChosenY);
      }
      else
      {
        *L_Pct_Strafe = 0.0;
      }
    }
    else if (VeADAS_t_DM_DebounceTime >= 0.3)
    {
      VeADAS_t_DM_DebounceTime = 0;
      *L_Pct_FwdRev = 0.0;
      *L_Pct_Strafe = 0.0;
    }  
    LeADAS_b_DM_StateComplete = true;
  }

  return (LeADAS_b_DM_StateComplete);
}