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
  // KV_ADAS_UT_LightDelayTIme = K_ADAS_UT_LightDelayTIme;
  // KV_ADAS_UT_LostTargetGx = K_ADAS_UT_LostTargetGx;
  // KV_ADAS_UT_NoTargetError = K_ADAS_UT_NoTargetError;
  // KV_ADAS_UT_DebounceTime = K_ADAS_UT_DebounceTime;
  // KV_ADAS_UT_AllowedLauncherError = K_ADAS_UT_AllowedLauncherError;
  // KV_ADAS_UT_AllowedLauncherTime = K_ADAS_UT_AllowedLauncherTime;
  // KV_ADAS_UT_RotateDeadbandAngle = K_ADAS_UT_RotateDeadbandAngle;
  // KV_ADAS_UT_TargetVisionAngle = K_ADAS_UT_TargetVisionAngle;

  // #ifdef ADAS_DM_Test
  // // display coefficients on SmartDashboard
  // frc::SmartDashboard::PutNumber("KV_ADAS_UT_LightDelayTIme", KV_ADAS_UT_LightDelayTIme);
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
  // KV_ADAS_UT_LightDelayTIme = frc::SmartDashboard::GetNumber("KV_ADAS_UT_LightDelayTIme", KV_ADAS_UT_LightDelayTIme);
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
  double L_Deg_TargetErrorActual = 0;

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

    if (V_GyroPrevious <= -170 && L_Deg_GyroAngleDeg >= 170 ||
        V_GyroFlipNeg && L_Deg_GyroAngleDeg < 0)
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


    L_Deg_TargetErrorActual = V_TargetAngle - L_Deg_GyroAngleDeg;
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
  double L_Deg_TargetErrorActual = 0;
  double L_Deg_GyroRolloverProtected = 0;

  *L_SD_RobotOriented = true;
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

  V_ADAS_DM_DebounceTime += C_ExeTime;

  if (V_ADAS_DM_DebounceTime <= K_ADAS_DM_DriveTimeLong)
    {
    *L_Pct_FwdRev = K_ADAS_DM_DriveFWD_Pct;
    }
  else
    {
    *L_Pct_FwdRev = 0;
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
  double L_ADAS_DM_ShooterSpeed;

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
                          double *L_RPM_Launcher,
                          double *L_Pct_Intake,
                          double *L_Pct_Elevator,
                          bool   *L_CameraUpperLightCmndOn,
                          bool   *L_CameraLowerLightCmndOn,
                          bool   *L_SD_RobotOriented,
                          double  L_L_X_FieldPos,
                          double  L_L_Y_FieldPos,
                          double  L_Deg_GyroAngleDeg,
                          int     L_i_PathNum)
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
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  /* Look up the desired target location point: */
  L_timeEND = DesiredAutonLocation2(V_ADAS_DM_StateTimer,
                                    L_i_PathNum,
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

  frc::SmartDashboard::PutNumber("Y relative pos",  L_L_RelativePosY);
  frc::SmartDashboard::PutNumber("X relative pos",  L_L_RelativePosX);
  frc::SmartDashboard::PutNumber("Y target pos",  L_L_TargetPositionY);
  frc::SmartDashboard::PutNumber("X target pos",  L_L_TargetPositionX);

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
    *L_Pct_Strafe =  Control_PID( L_L_TargetPositionX,
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

     *L_Pct_FwdRev =  Control_PID( L_L_TargetPositionY,
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

      ADAS_DM_FieldOrientRotate(L_Pct_FwdRev,
                                L_Pct_Strafe,
                                L_Pct_Rotate,
                                L_RPM_Launcher,
                                L_Pct_Intake,
                                L_Pct_Elevator,
                                L_CameraUpperLightCmndOn,
                                L_CameraLowerLightCmndOn,
                                L_SD_RobotOriented,
                                L_Deg_GyroAngleDeg,
                                L_Deg_RotateTarget);

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

  return (L_ADAS_DM_StateComplete);
  }