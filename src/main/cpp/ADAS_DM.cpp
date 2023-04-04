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

double VeADAS_t_DM_Debounce = 0;
bool VeADAS_b_DM_StateInit = false;
double V_ADAS_DM_Rotate180TargetAngle = 0;
double V_ADAS_DM_InitGyroAngle = 0;
double VeADAS_t_DM_StateTimer = 0;
double V_ADAS_DM_X_ErrorPrev = 0;
double V_ADAS_DM_Y_ErrorPrev = 0;
double VeADAS_Deg_DM_AngleErrorPrev = 0;
double V_ADAS_DM_X_Integral = 0;
double V_ADAS_DM_Y_Integral = 0;
double VeADAS_Deg_DM_AngleIntegral = 0;
double VeADAS_l_DM_X_StartPosition = 0;
double VeADAS_l_DM_Y_StartPosition = 0;
double VeADAS_Deg_DM_StartAng = 0;
double VeADAS_Deg_DM_TargetStartAng = 0;
double VeADAS_l_DM_X_TargetStartPosition = 0;
double VeADAS_l_DM_Y_TargetStartPosition = 0;
double V_ADAS_DM_TargetAngle;
double V_ADAS_DM_GyroPrevious;
bool V_ADAS_DM_GyroFlipNeg;
bool V_ADAS_DM_GyroFlipPos;

bool VeADAS_b_DM_AutoBalanceInit = false;
bool VeADAS_b_DM_AutoBalancePositive = false;
bool VeADAS_b_DM_AutoBalanceFastSearch = false;
double VeADAS_t_DM_AutoBalanceDbTm = 0.0;
double VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
double VeADAS_Deg_DM_AutoBalanceErrorPrev = 0;
double VeADAS_Deg_DM_AutoBalanceIntegral = 0;

double VeADAS_t_DM_AutoMountDbTime = 0;
TeADAS_DM_DriveOverStation VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_FwdFlat1;

double VeADAS_t_DM_DebounceTime = 0;

double V_TargetAngle;
double V_GyroPrevious;
bool V_GyroFlipNeg;
bool V_GyroFlipPos;
double V_OffsettedGyro;

bool wantToStopX;
bool wantToStopY;

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

#endif
}

/******************************************************************************
 * Function:     ADAS_DM_Reset
 *
 * Description:  Reset all applicable DM variables.
 ******************************************************************************/
void ADAS_DM_Reset(void)
{
  VeADAS_t_DM_Debounce = 0;
  VeADAS_b_DM_StateInit = false;
  V_ADAS_DM_Rotate180TargetAngle = 0;
  V_ADAS_DM_InitGyroAngle = 0;
  VeADAS_t_DM_StateTimer = 0;
  V_ADAS_DM_X_ErrorPrev = 0;
  V_ADAS_DM_Y_ErrorPrev = 0;
  VeADAS_Deg_DM_AngleErrorPrev = 0;
  V_ADAS_DM_X_Integral = 0;
  V_ADAS_DM_Y_Integral = 0;
  VeADAS_Deg_DM_AngleIntegral = 0;
  VeADAS_l_DM_X_StartPosition = 0;
  VeADAS_l_DM_Y_StartPosition = 0;
  VeADAS_Deg_DM_StartAng = 0;
  VeADAS_Deg_DM_TargetStartAng = 0;
  VeADAS_l_DM_X_TargetStartPosition = 0;
  VeADAS_l_DM_Y_TargetStartPosition = 0;
  VeADAS_b_DM_AutoBalanceInit = false;
  VeADAS_b_DM_AutoBalancePositive = false;
  VeADAS_b_DM_AutoBalanceFastSearch = false;
  VeADAS_t_DM_AutoBalanceDbTm = 0.0;
  VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
  VeADAS_Deg_DM_AutoBalanceErrorPrev = 0;
  VeADAS_Deg_DM_AutoBalanceIntegral = 0;
  VeADAS_t_DM_AutoMountDbTime = 0.0;
  VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_FwdFlat1;
}

/******************************************************************************
 * Function:     ADAS_DM_Rotate180
 *
 * Description:  Rotate 180 degrees control.
 ******************************************************************************/
bool ADAS_DM_Rotate180(double *LeADAS_Pct_FwdRev,
                       double *LeADAS_Pct_Strafe,
                       double *LeADAS_Pct_Rotate,
                       double *L_RPM_Launcher,
                       double *L_Pct_Intake,
                       double *L_Pct_Elevator,
                       bool *L_CameraUpperLightCmndOn,
                       bool *L_CameraLowerLightCmndOn,
                       bool *LeADAS_b_SD_RobotOriented,
                       double L_Deg_GyroAngleDeg)
{
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;

  *LeADAS_b_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *LeADAS_Pct_FwdRev = 0;
  *LeADAS_Pct_Strafe = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  if (VeADAS_b_DM_StateInit == false)
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

    VeADAS_b_DM_StateInit = true;
  }

  L_RotateError = V_ADAS_DM_Rotate180TargetAngle - L_Deg_GyroAngleDeg;

  if (fabs(L_RotateError) <= K_ADAS_DM_RotateDeadbandAngle && VeADAS_t_DM_Debounce < K_ADAS_DM_RotateDebounceTime)
  {
    VeADAS_t_DM_Debounce += C_ExeTime;
  }
  else if (fabs(L_RotateError) > K_ADAS_DM_RotateDeadbandAngle)
  {
    /* Reset the timer, we have gone out of bounds */
    VeADAS_t_DM_Debounce = 0;
  }
  else if (VeADAS_t_DM_Debounce >= K_ADAS_DM_RotateDebounceTime)
  {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    VeADAS_t_DM_Debounce = 0;
  }

  if (L_ADAS_DM_StateComplete == false)
  {
    *LeADAS_Pct_Rotate = DesiredRotateSpeed(L_RotateError);
  }
  else
  {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *LeADAS_Pct_Rotate = 0;
    VeADAS_t_DM_Debounce = 0;
    L_ADAS_DM_StateComplete = true;
    VeADAS_b_DM_StateInit = false;
  }

  return (L_ADAS_DM_StateComplete);
}
/******************************************************************************
 * Function:     ADAS_DM_RotateTo
 *
 * Description:  Rotate to a given position (L_Deg_GyroAngleTarget)
 ******************************************************************************/
bool ADAS_DM_RotateTo(double *LeADAS_Pct_FwdRev,
                      double *LeADAS_Pct_Strafe,
                      double *LeADAS_Pct_Rotate,
                      double *L_RPM_Launcher,
                      double *L_Pct_Intake,
                      double *L_Pct_Elevator,
                      bool *L_CameraUpperLightCmndOn,
                      bool *L_CameraLowerLightCmndOn,
                      bool *LeADAS_b_SD_RobotOriented,
                      double L_Deg_GyroAngleDeg,
                      double L_Deg_GyroAngleTarget)
{
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;
  double L_ClockwiseError = 0;
  double L_CounterClockwiseError = 0;

  *LeADAS_b_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *L_CameraUpperLightCmndOn = false;
  *L_CameraLowerLightCmndOn = false;
  *LeADAS_Pct_FwdRev = 0;
  *LeADAS_Pct_Strafe = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;

  if (VeADAS_b_DM_StateInit == false)
  {
    /* Need to find the target angle.  The gyro in use will only report values of -180 to 180. Need to account for this:*/
    // V_ADAS_DM_Rotate180TargetAngle = L_InitGyroAngle - 180;

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
    else if (V_GyroPrevious >= 170 && L_Deg_GyroAngleDeg <= 170)
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

    VeADAS_b_DM_StateInit = true;
  }

  L_RotateError = V_ADAS_DM_Rotate180TargetAngle - L_Deg_GyroAngleDeg;

  if (fabs(L_RotateError) <= K_ADAS_DM_RotateDeadbandAngle && VeADAS_t_DM_Debounce < K_ADAS_DM_RotateDebounceTime)
  {
    VeADAS_t_DM_Debounce += C_ExeTime;
  }
  else if (fabs(L_RotateError) > K_ADAS_DM_RotateDeadbandAngle)
  {
    /* Reset the timer, we have gone out of bounds */
    VeADAS_t_DM_Debounce = 0;
  }
  else if (VeADAS_t_DM_Debounce >= K_ADAS_DM_RotateDebounceTime)
  {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    VeADAS_t_DM_Debounce = 0;
  }

  if (L_ADAS_DM_StateComplete == false)
  {
    *LeADAS_Pct_Rotate = DesiredRotateSpeed(L_RotateError);
  }
  else
  {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *LeADAS_Pct_Rotate = 0;
    VeADAS_t_DM_Debounce = 0;
    L_ADAS_DM_StateComplete = true;
    VeADAS_b_DM_StateInit = false;
  }

  return (L_ADAS_DM_StateComplete);
}

/******************************************************************************
 * Function:     ADAS_DM_FieldOrientRotate
 *
 * Description:  Rotate to the zeroed position
 ******************************************************************************/
bool ADAS_DM_FieldOrientRotate(double *LeADAS_Pct_FwdRev,
                               double *LeADAS_Pct_Strafe,
                               double *LeADAS_Pct_Rotate,
                               double *L_Pct_Intake,
                               bool *LeADAS_b_SD_RobotOriented,
                               double L_Deg_GyroAngleDeg,
                               double L_Deg_GyroAngleTarget)
{
  bool L_ADAS_DM_StateComplete = false;
  double L_RotateError = 0;
  double L_Deg_TargetErrorActual = 0;
  double L_Deg_GyroRolloverProtected = 0;

  *LeADAS_b_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *LeADAS_Pct_FwdRev = 0;
  *LeADAS_Pct_Strafe = 0;
  *L_Pct_Intake = 0;

  if (VeADAS_b_DM_StateInit == false)
  {
    /* Need to find the target angle.  The gyro in use will only report values of -180 to 180. Need to account for this:*/
    // V_ADAS_DM_Rotate180TargetAngle = L_InitGyroAngle - 180;

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
    VeADAS_b_DM_StateInit = true;
  }

  L_RotateError = V_TargetAngle - L_Deg_GyroAngleDeg;

  /* Detect Gyro roll over. */
  if ((V_ADAS_DM_GyroPrevious <= -170 && L_Deg_GyroAngleDeg >= 170) ||
      (V_ADAS_DM_GyroFlipNeg == true && L_Deg_GyroAngleDeg > 0))
  {
    L_Deg_GyroRolloverProtected = L_Deg_GyroAngleDeg - 360;
    V_ADAS_DM_GyroFlipNeg = true;
    V_ADAS_DM_GyroFlipPos = false;
  }
  else if ((V_GyroPrevious >= 170 && L_Deg_GyroAngleDeg <= 170) ||
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

  if (fabs(L_Deg_TargetErrorActual) <= K_ADAS_DM_RotateDeadbandAngle && VeADAS_t_DM_Debounce < K_ADAS_DM_RotateDebounceTime)
  {
    VeADAS_t_DM_Debounce += C_ExeTime;
  }
  else if (fabs(L_Deg_TargetErrorActual) > K_ADAS_DM_RotateDeadbandAngle)
  {
    /* Reset the timer, we have gone out of bounds */
    VeADAS_t_DM_Debounce = 0;
  }
  else if (VeADAS_t_DM_Debounce >= K_ADAS_DM_RotateDebounceTime)
  {
    /* Reset the time, proceed to next state. */
    L_ADAS_DM_StateComplete = true;
    VeADAS_t_DM_Debounce = 0;
  }

  if (L_ADAS_DM_StateComplete == false)
  {
    *LeADAS_Pct_Rotate = DesiredRotateSpeed(L_Deg_TargetErrorActual);
  }
  else
  {
    /* We have been at the correct location for the set amount of time.
       We have previously set the state to the next one, now set the rotate command to off. */
    *LeADAS_Pct_Rotate = 0;
    VeADAS_t_DM_Debounce = 0;
    L_ADAS_DM_StateComplete = true;
    VeADAS_b_DM_StateInit = false;
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
bool ADAS_DM_DriveStraight(double *LeADAS_Pct_FwdRev,
                           double *LeADAS_Pct_Strafe,
                           double *LeADAS_Pct_Rotate,
                           bool   *LeADAS_b_SD_RobotOriented)
{
  bool L_ADAS_DM_StateComplete = false;

  *LeADAS_b_SD_RobotOriented = true;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *LeADAS_Pct_Strafe = 0;
  *LeADAS_Pct_Rotate = 0;

  VeADAS_t_DM_Debounce += C_ExeTime; // update our timekeeping

  if (VeADAS_t_DM_Debounce <= K_ADAS_DM_DriveTimeLong) // check that we are still in the time we have given ourselves
  {
    *LeADAS_Pct_FwdRev = K_ADAS_DM_DriveFWD_Pct; // set our strafe percent to this constant
  }
  else
  {
    *LeADAS_Pct_FwdRev = 0; // reset all the variables a driver state
    *LeADAS_b_SD_RobotOriented = false;
    VeADAS_t_DM_Debounce = 0;
    L_ADAS_DM_StateComplete = true;
  }
  return (L_ADAS_DM_StateComplete);
}

/******************************************************************************
 * Function:     ADAS_DM_DriveStraightFar
 *
 * Description:  Drive straight control, driving past the line.
 ******************************************************************************/
bool ADAS_DM_DriveStraightFar(double *LeADAS_Pct_FwdRev,
                              double *LeADAS_Pct_Strafe,
                              double *LeADAS_Pct_Rotate,
                              bool   *LeADAS_b_SD_RobotOriented)
{
  bool L_ADAS_DM_StateComplete = false;

  *LeADAS_b_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *LeADAS_Pct_Strafe = 0;
  *LeADAS_Pct_Rotate = 0;

  VeADAS_t_DM_Debounce += C_ExeTime; // update our timekeeping

  if (VeADAS_t_DM_Debounce <= KeADAS_t_DM_DriveTimeFar) // check that we are still in the time we have given ourselves
  {
    *LeADAS_Pct_FwdRev = KeADAS_Pct_DM_DriveFWD_Far; // set our drive percent to this constant
  }
  else
  {
    *LeADAS_Pct_FwdRev = 0; // reset all the variables a driver state
    *LeADAS_b_SD_RobotOriented = false;
    VeADAS_t_DM_Debounce = 0;
    L_ADAS_DM_StateComplete = true;
  }
  return (L_ADAS_DM_StateComplete);
}

/******************************************************************************
 * Function:     ADAS_DM_DriveRevStraight
 *
 * Description:  Like drive straight control, but in reverse!
 ******************************************************************************/
bool ADAS_DM_DriveRevStraight(double *LeADAS_Pct_FwdRev,
                              double *LeADAS_Pct_Strafe,
                              double *LeADAS_Pct_Rotate,
                              bool   *LeADAS_b_SD_RobotOriented,
                              bool    LeADAS_b_CompletePrev)
{
  bool L_ADAS_DM_StateComplete = false;

  *LeADAS_b_SD_RobotOriented = false;
  /* Next, let's set all the other items we aren't trying to control to off: */
  *LeADAS_Pct_Strafe = 0;
  *LeADAS_Pct_Rotate = 0;
  
  if (LeADAS_b_CompletePrev == false)
  {
    VeADAS_t_DM_Debounce += C_ExeTime; // update our timekeeping
  }
  

  if ((VeADAS_t_DM_Debounce <= KeADAS_t_DM_RevDriveTime) && (LeADAS_b_CompletePrev == false)) // check that we are still in the time we have given ourselves
  {
    *LeADAS_Pct_FwdRev = KeADAS_Pct_DM_RevDrive; // set our strafe percent to this constant
  }
  else
  {
    *LeADAS_Pct_FwdRev = 0; // reset all the variables a driver state
    *LeADAS_b_SD_RobotOriented = false;
    VeADAS_t_DM_Debounce = 0;
    L_ADAS_DM_StateComplete = true;
  }
  return (L_ADAS_DM_StateComplete);
}

/******************************************************************************
 * Function:     ADAS_DM_DriveRevStraight
 *
 * Description:  Like drive straight control, but in reverse!
 ******************************************************************************/
bool ADAS_DM_Stop(double *LeADAS_Pct_FwdRev,
                  double *LeADAS_Pct_Strafe,
                  double *LeADAS_Pct_Rotate,
                  bool   *LeADAS_b_SD_RobotOriented,
                  double  LeADAS_t_StopTime)
{
  bool L_ADAS_DM_StateComplete = false;

  *LeADAS_b_SD_RobotOriented = false;
  *LeADAS_Pct_Strafe = 0;
  *LeADAS_Pct_FwdRev = 0;
  *LeADAS_Pct_Rotate = 0;
  

  VeADAS_t_DM_Debounce += C_ExeTime; // update our timekeeping

  if (VeADAS_t_DM_Debounce >= LeADAS_t_StopTime)
  {
    L_ADAS_DM_StateComplete = true;
    VeADAS_t_DM_Debounce = 0;
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
bool ADAS_DM_PathFollower(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          double *LeADAS_Deg_DesiredPose,
                          bool   *LeADAS_b_SD_RobotOriented,
                          double  LeADAS_l_X_FieldPos,
                          double  LeADAS_l_Y_FieldPos,
                          double  LeADAS_Deg_GyroAngle,
                          int     LeADAS_i_PathNum)
{
  bool   LeADAS_b_DM_StateComplete = false;
  double LeADAS_l_TargetPositionX = 0.0;
  double LeADAS_l_TargetPositionY = 0.0;
  double LeADAS_Deg_TargetAngle = 0.0;
  double LeADAS_l_RelativePosX = 0.0;
  double LeADAS_l_RelativePosY = 0.0;
  double LeADAS_Deg_RelativeAng = 0.0;
  double LeADAS_l_X_Error = 0.0;
  double LeADAS_l_Y_Error = 0.0;
  double LeADAS_Deg_RotateError = 0.0;
  double LeADAS_Deg_RotateErrorRaw = 0.0;
  double LeADAS_t_TimeReaining = 0.0;
  bool   LeADAS_b_TimeEndReached = false;
  double LeADAS_k_SlowSearch = 1.0;

  /* Set the things we are not using to off: */
  *LeADAS_b_SD_RobotOriented = false;

  /* Look up the desired target location point: */
  LeADAS_b_TimeEndReached = DesiredAutonLocation2( VeADAS_t_DM_StateTimer,
                                                   LeADAS_i_PathNum,
                                                  &LeADAS_l_TargetPositionX,
                                                  &LeADAS_l_TargetPositionY,
                                                  &LeADAS_Deg_TargetAngle,
                                                  &LeADAS_t_TimeReaining);

  /* Capture some of the things we need to save for this state control: */
  if (VeADAS_b_DM_StateInit == false)
  {
    VeADAS_l_DM_X_StartPosition = LeADAS_l_X_FieldPos;
    VeADAS_l_DM_Y_StartPosition = LeADAS_l_Y_FieldPos;
    VeADAS_l_DM_X_TargetStartPosition = LeADAS_l_TargetPositionX;
    VeADAS_l_DM_Y_TargetStartPosition = LeADAS_l_TargetPositionY;
    VeADAS_Deg_DM_StartAng = LeADAS_Deg_GyroAngle;
    VeADAS_Deg_DM_TargetStartAng = LeADAS_Deg_TargetAngle;
    VeADAS_b_DM_StateInit = true;
  }

  /* We need to offset the position by the start position since the odometry will
     start at zero, but the lookup table will not */
  LeADAS_l_TargetPositionX -= VeADAS_l_DM_X_TargetStartPosition;
  LeADAS_l_TargetPositionY -= VeADAS_l_DM_Y_TargetStartPosition;
  LeADAS_Deg_TargetAngle   -= VeADAS_Deg_DM_TargetStartAng;

  LeADAS_l_RelativePosX = LeADAS_l_X_FieldPos - VeADAS_l_DM_X_StartPosition;
  LeADAS_l_RelativePosY = LeADAS_l_Y_FieldPos - VeADAS_l_DM_Y_StartPosition;
  LeADAS_Deg_RelativeAng = LeADAS_Deg_GyroAngle - VeADAS_Deg_DM_StartAng;

  LeADAS_l_X_Error       = fabs(LeADAS_l_TargetPositionX - LeADAS_l_RelativePosX);
  LeADAS_l_Y_Error       = fabs(LeADAS_l_TargetPositionY - LeADAS_l_RelativePosY);
  // LeADAS_Deg_RotateError = fabs(LeADAS_Deg_TargetAngle - LeADAS_Deg_RelativeAng);
  
  VeADAS_t_DM_StateTimer += C_ExeTime;

  LeADAS_Deg_RotateErrorRaw = LeADAS_Deg_TargetAngle - LeADAS_Deg_RelativeAng;

  if (LeADAS_Deg_RotateErrorRaw < -180)
    {
    LeADAS_Deg_TargetAngle += 360;
    LeADAS_Deg_RotateError = LeADAS_Deg_RotateErrorRaw + 360;
    }
  else if (LeADAS_Deg_RotateErrorRaw > 180)
    {
    LeADAS_Deg_TargetAngle -= 360;
    LeADAS_Deg_RotateError = LeADAS_Deg_RotateErrorRaw - 360;
    }
  else
    {
    LeADAS_Deg_RotateError = LeADAS_Deg_RotateErrorRaw;
    }

  /* Exit criteria: */
  if (fabs(LeADAS_Deg_RotateError) <= K_ADAS_DM_RotateDeadbandAngle &&
      LeADAS_l_X_Error <= K_ADAS_DM_XY_Deadband &&
      LeADAS_l_Y_Error <= K_ADAS_DM_XY_Deadband &&
      VeADAS_t_DM_Debounce < KeADAS_t_DM_PathFollowDebounceTime &&
      LeADAS_b_TimeEndReached == true)
  {
    VeADAS_t_DM_Debounce += C_ExeTime;
    // LeADAS_Deg_RotateError = 0.0;
  }
  else if (VeADAS_t_DM_Debounce >= KeADAS_t_DM_PathFollowDebounceTime)
  {
    /* Reset the time, proceed to next state. */
    LeADAS_b_DM_StateComplete = true;
    VeADAS_t_DM_Debounce = 0;
  }
  else if (fabs(LeADAS_Deg_RotateError) > K_ADAS_DM_RotateDeadbandAngle ||
           LeADAS_l_X_Error > K_ADAS_DM_XY_Deadband ||
           LeADAS_l_Y_Error > K_ADAS_DM_XY_Deadband)
  {
    /* Reset the timer, we have gone out of bounds */
    VeADAS_t_DM_Debounce = 0;
  }
  
  if (LeADAS_t_TimeReaining < 0.05)
  {
    // LeADAS_k_SlowSearch = 0.7;
  }
  
  if (LeADAS_b_DM_StateComplete == false)
  {
    *LeADAS_Pct_Strafe = -Control_PID(LeADAS_l_TargetPositionX,
                                      LeADAS_l_RelativePosX,
                                      &V_ADAS_DM_X_ErrorPrev,
                                      &V_ADAS_DM_X_Integral,
                                      K_k_AutonX_PID_Gx[E_P_Gx] * LeADAS_k_SlowSearch,
                                      K_k_AutonX_PID_Gx[E_I_Gx] * LeADAS_k_SlowSearch,
                                      K_k_AutonX_PID_Gx[E_D_Gx] * LeADAS_k_SlowSearch,
                                      K_k_AutonX_PID_Gx[E_P_Ul],
                                      K_k_AutonX_PID_Gx[E_P_Ll],
                                      K_k_AutonX_PID_Gx[E_I_Ul],
                                      K_k_AutonX_PID_Gx[E_I_Ll],
                                      K_k_AutonX_PID_Gx[E_D_Ul],
                                      K_k_AutonX_PID_Gx[E_D_Ll],
                                      K_k_AutonX_PID_Gx[E_Max_Ul],
                                      K_k_AutonX_PID_Gx[E_Max_Ll]);

    *LeADAS_Pct_FwdRev = -Control_PID(LeADAS_l_TargetPositionY,
                                      LeADAS_l_RelativePosY,
                                      &V_ADAS_DM_Y_ErrorPrev,
                                      &V_ADAS_DM_Y_Integral,
                                      K_k_AutonY_PID_Gx[E_P_Gx] * LeADAS_k_SlowSearch,
                                      K_k_AutonY_PID_Gx[E_I_Gx] * LeADAS_k_SlowSearch,
                                      K_k_AutonY_PID_Gx[E_D_Gx] * LeADAS_k_SlowSearch,
                                      K_k_AutonY_PID_Gx[E_P_Ul],
                                      K_k_AutonY_PID_Gx[E_P_Ll],
                                      K_k_AutonY_PID_Gx[E_I_Ul],
                                      K_k_AutonY_PID_Gx[E_I_Ll],
                                      K_k_AutonY_PID_Gx[E_D_Ul],
                                      K_k_AutonY_PID_Gx[E_D_Ll],
                                      K_k_AutonY_PID_Gx[E_Max_Ul],
                                      K_k_AutonY_PID_Gx[E_Max_Ll]);

    *LeADAS_Pct_Rotate =  Control_PID(LeADAS_Deg_TargetAngle,
                                      LeADAS_Deg_RelativeAng,
                                      &VeADAS_Deg_DM_AngleErrorPrev,
                                      &VeADAS_Deg_DM_AngleIntegral,
                                      KeADAS_k_AutonRotatePID_Gx[E_P_Gx] * LeADAS_k_SlowSearch,
                                      KeADAS_k_AutonRotatePID_Gx[E_I_Gx] * LeADAS_k_SlowSearch,
                                      KeADAS_k_AutonRotatePID_Gx[E_D_Gx] * LeADAS_k_SlowSearch,
                                      KeADAS_k_AutonRotatePID_Gx[E_P_Ul],
                                      KeADAS_k_AutonRotatePID_Gx[E_P_Ll],
                                      KeADAS_k_AutonRotatePID_Gx[E_I_Ul],
                                      KeADAS_k_AutonRotatePID_Gx[E_I_Ll],
                                      KeADAS_k_AutonRotatePID_Gx[E_D_Ul],
                                      KeADAS_k_AutonRotatePID_Gx[E_D_Ll],
                                      KeADAS_k_AutonRotatePID_Gx[E_Max_Ul],
                                      KeADAS_k_AutonRotatePID_Gx[E_Max_Ll]);

    *LeADAS_Deg_DesiredPose = LeADAS_Deg_TargetAngle;
  }
  else
  {
    /* We have been at the correct location for the set amount of time. */
    *LeADAS_Pct_FwdRev = 0;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    VeADAS_t_DM_Debounce = 0;
    VeADAS_t_DM_StateTimer = 0;
    LeADAS_b_DM_StateComplete = true;
    VeADAS_b_DM_StateInit = false;
    VeADAS_l_DM_X_StartPosition = 0;
    VeADAS_l_DM_Y_StartPosition = 0;
    VeADAS_l_DM_X_TargetStartPosition = 0;
    VeADAS_l_DM_Y_TargetStartPosition = 0;
    V_ADAS_DM_X_ErrorPrev = 0;
    V_ADAS_DM_X_Integral = 0;
    V_ADAS_DM_Y_ErrorPrev = 0;
    V_ADAS_DM_Y_Integral = 0;
    VeADAS_Deg_DM_AngleErrorPrev = 0;
    VeADAS_Deg_DM_AngleIntegral = 0;
  }

  frc::SmartDashboard::PutNumber("DM Timer", VeADAS_t_DM_StateTimer);
  frc::SmartDashboard::PutNumber("X Cmnd",   LeADAS_l_TargetPositionX);
  frc::SmartDashboard::PutNumber("X Act",    LeADAS_l_RelativePosX);
  frc::SmartDashboard::PutNumber("X Err",    LeADAS_l_X_Error);
  frc::SmartDashboard::PutNumber("Y Cmnd",   LeADAS_l_TargetPositionY);
  frc::SmartDashboard::PutNumber("Y Act",    LeADAS_l_RelativePosY);
  frc::SmartDashboard::PutNumber("Y Err",    LeADAS_l_Y_Error);
  frc::SmartDashboard::PutNumber("Rot Cmnd", LeADAS_Deg_TargetAngle);
  frc::SmartDashboard::PutNumber("Rot Act",  LeADAS_Deg_RelativeAng);
  frc::SmartDashboard::PutNumber("Rot Err",  LeADAS_Deg_RotateError);

  return (LeADAS_b_DM_StateComplete);
}

/******************************************************************************
 * Function:     ADAS_DM_AutoBalance
 *
 * Description:  Balance on the charge station
 ******************************************************************************/
bool ADAS_DM_AutoBalance(double *LeADAS_Pct_FwdRev,
                         double *LeADAS_Pct_Strafe,
                         double *LeADAS_Pct_Rotate,
                         bool *LeADAS_b_SD_RobotOriented,
                         bool *LeADAS_b_X_Mode,
                         double LeADAS_Deg_GyroRoll)
{
  bool LeADAS_b_DM_StateComplete = false;
  double LeADAS_Deg_RollError = 0;
  T_PID_Cal L_Index = E_P_Gx;
  double LeADAS_k_DM_AutoBalancePID[E_PID_CalSz];
  bool    LeADAS_b_Searching = false;

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
    VeADAS_b_DM_AutoBalanceInit = true;  // This is intended on being here, but need to verify if this will break the PID....
  }

  /* Detect roll over here: */
  if (VeADAS_b_DM_AutoBalanceFastSearch == true &&
      (((VeADAS_b_DM_AutoBalancePositive == true) && (LeADAS_Deg_GyroRoll < 0)) ||
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
      (VeADAS_t_DM_AutoBalanceDbTm < KeADAS_t_DM_AutoBalanceDb))
  {
    VeADAS_t_DM_AutoBalanceDbTm += C_ExeTime;
    VeADAS_Deg_DM_AutoBalanceErrorPrev = 0.0;
    VeADAS_Deg_DM_AutoBalanceIntegral = 0.0;
    LeADAS_b_Searching = false;
  }
  else if (fabs(LeADAS_Deg_RollError) > KeADAS_Deg_DM_AutoBalanceDb)
  {
    /* Reset the timer, we have gone out of bounds */
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    LeADAS_b_Searching = true;
  }
  else if (VeADAS_t_DM_AutoBalanceDbTm >= KeADAS_Deg_DM_AutoBalanceDb)
  {
    /* Reset the time, proceed to next state. */
    LeADAS_b_DM_StateComplete = true;
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    VeADAS_t_DM_AutoBalanceHoldTm += C_ExeTime;
    VeADAS_Deg_DM_AutoBalanceErrorPrev = 0.0;
    VeADAS_Deg_DM_AutoBalanceIntegral = 0.0;
    LeADAS_b_Searching = false;
  }

  if ((LeADAS_b_Searching == true))
  {
    *LeADAS_Pct_FwdRev = Control_PID(0.0,
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

    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = true;
    *LeADAS_b_X_Mode = false;
  }
  else if ((VeADAS_t_DM_AutoBalanceHoldTm <= KeADAS_t_DM_AutoBalanceHold))
  {
    *LeADAS_Pct_FwdRev = 0;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = true;
    VeADAS_t_DM_AutoBalanceHoldTm += C_ExeTime;
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    LeADAS_b_DM_StateComplete = false;
    VeADAS_b_DM_AutoBalanceInit = false;
  }
  else
  {
    /* We have been at the correct location for the set amount of time. */
    *LeADAS_Pct_FwdRev = 0;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = true;
    VeADAS_t_DM_AutoBalanceDbTm = 0;
    VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
    LeADAS_b_DM_StateComplete = true;
    VeADAS_b_DM_AutoBalanceInit = false;
  }

  return (LeADAS_b_DM_StateComplete);
}


/******************************************************************************
 * Function:     ADAS_DM_DriveOntoStation
 *
 * Description:  Drive over the charge station, reverse and mount
 ******************************************************************************/
bool ADAS_DM_DriveOntoStation(double *LeADAS_Pct_FwdRev,
                              double *LeADAS_Pct_Strafe,
                              double *LeADAS_Pct_Rotate,
                              bool   *LeADAS_b_SD_RobotOriented,
                              bool   *LeADAS_b_X_Mode,
                              double  LeADAS_Deg_GyroRoll)
{
  bool LeADAS_b_DM_StateComplete = false;
  double LeADAS_Pct_Drive = 0.0;

  /* Look up the desired target location point: */

  /* Capture some of the things we need to save for this state control: */

  if (VeADAS_e_DM_AutoMountState == E_ADAS_DM_DriveOS_FwdFlat1)
    {
      LeADAS_Pct_Drive = KeADAS_Pct_DM_AutoMountPwr;

      if (LeADAS_Deg_GyroRoll > KeADAS_Deg_DM_AutoMountDetect)  // need to verify direction
        {
          VeADAS_t_DM_AutoMountDbTime += C_ExeTime;
        }
      if (VeADAS_t_DM_AutoMountDbTime >= KeADAS_t_DM_AutoMountDb)
        {
          VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_FwdRampUp;
          VeADAS_t_DM_AutoMountDbTime = 0.0;
        }
    }
  else if (VeADAS_e_DM_AutoMountState == E_ADAS_DM_DriveOS_FwdRampUp)
    {
      LeADAS_Pct_Drive = KeADAS_Pct_DM_AutoMountPwr;
      
      if (LeADAS_Deg_GyroRoll < -KeADAS_Deg_DM_AutoMountDetect)  // need to verify direction
        {
          VeADAS_t_DM_AutoMountDbTime += C_ExeTime;
        }
      if (VeADAS_t_DM_AutoMountDbTime >= KeADAS_t_DM_AutoMountDb)
        {
          VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_FwdRampDwn;
          VeADAS_t_DM_AutoMountDbTime = 0.0;
        }
    }
  else if (VeADAS_e_DM_AutoMountState == E_ADAS_DM_DriveOS_FwdRampDwn)
    {
      LeADAS_Pct_Drive = KeADAS_Pct_DM_AutoMountPwr;

      if ((LeADAS_Deg_GyroRoll > -KeADAS_Deg_DM_AutoMountDetect) && (LeADAS_Deg_GyroRoll < KeADAS_Deg_DM_AutoMountDetect)) // need to verify direction
        {
          VeADAS_t_DM_AutoMountDbTime += C_ExeTime;
        }
      if (VeADAS_t_DM_AutoMountDbTime >= KeADAS_t_DM_AutoMountRevDb)
        {
          VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_RevRampUp;
          VeADAS_t_DM_AutoMountDbTime = 0.0;
        }
    }
  else if (VeADAS_e_DM_AutoMountState == E_ADAS_DM_DriveOS_RevRampUp)
    {
      LeADAS_Pct_Drive = -KeADAS_Pct_DM_AutoMountPwr;

      if (LeADAS_Deg_GyroRoll < -KeADAS_Deg_DM_AutoMountDetect) // need to verify direction
        {
          VeADAS_t_DM_AutoMountDbTime += C_ExeTime;
        }
      if (VeADAS_t_DM_AutoMountDbTime >= KeADAS_t_DM_AutoMountDb)
        {
          VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_Complete;
          VeADAS_t_DM_AutoMountDbTime = 0.0;
          LeADAS_b_DM_StateComplete = true;
        }
    }

  /* Exit criteria: */
  if (LeADAS_b_DM_StateComplete == false)
  {
    *LeADAS_Pct_FwdRev = LeADAS_Pct_Drive;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = false;
  }
  else
  {
    /* We have been at the correct location for the set amount of time. */
    *LeADAS_Pct_FwdRev = 0;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = false;
    VeADAS_t_DM_AutoMountDbTime = 0;
  }
  return (LeADAS_b_DM_StateComplete);
}


/******************************************************************************
 * Function:     ADAS_DM_MountStation
 *
 * Description:  Drive onto the the charge station
 ******************************************************************************/
bool ADAS_DM_MountStation(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          bool   *LeADAS_b_SD_RobotOriented,
                          bool   *LeADAS_b_X_Mode,
                          double  LeADAS_Deg_GyroRoll)
{
  bool LeADAS_b_DM_StateComplete = false;
  double LeADAS_Pct_Drive = 0.0;

  /* Look up the desired target location point: */

  /* Capture some of the things we need to save for this state control: */

  if (VeADAS_e_DM_AutoMountState == E_ADAS_DM_DriveOS_FwdFlat1)
    {
      LeADAS_Pct_Drive = KeADAS_Pct_DM_AutoMountPwr;

      if (LeADAS_Deg_GyroRoll > KeADAS_Deg_DM_AutoMountDetect)  // need to verify direction
        {
          VeADAS_t_DM_AutoMountDbTime += C_ExeTime;
        }
      if (VeADAS_t_DM_AutoMountDbTime >= KeADAS_t_DM_AutoMountOnlyDb)
        {
          VeADAS_e_DM_AutoMountState = E_ADAS_DM_DriveOS_Complete;
          VeADAS_t_DM_AutoMountDbTime = 0.0;
          LeADAS_b_DM_StateComplete = true;
        }
    }

  /* Exit criteria: */
  if (LeADAS_b_DM_StateComplete == false)
  {
    *LeADAS_Pct_FwdRev = LeADAS_Pct_Drive;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = false;
  }
  else
  {
    /* We have been at the correct location for the set amount of time. */
    *LeADAS_Pct_FwdRev = 0;
    *LeADAS_Pct_Strafe = 0;
    *LeADAS_Pct_Rotate = 0;
    *LeADAS_b_SD_RobotOriented = false;
    *LeADAS_b_X_Mode = false;
    VeADAS_t_DM_AutoMountDbTime = 0;
  }

  return (LeADAS_b_DM_StateComplete);
}


/******************************************************************************
 * Function:     MoveWithOffsetTag
 *
 * Description:  TBD
 ******************************************************************************/
bool MoveWithOffsetTag(double *LeADAS_Pct_FwdRev,
                       double *LeADAS_Pct_Strafe,
                       double *LeADAS_Pct_Rotate,
                       bool L_OdomCentered,
                       double L_TagYawDegrees,
                       double L_OdomOffsetX,
                       double L_OdomOffsetY,
                       double L_RequestedOffsetX,
                       double L_RequestedOffsetY)
{
  bool LeADAS_b_DM_StateComplete = false;
  double L_ErrorCalcYaw;
  
  // if (L_OdomCentered)
  // {

    L_ErrorCalcYaw = 0.18 - L_TagYawDegrees; // -.18 is just what it happened to be idk

    if (L_ErrorCalcYaw > 0 || L_ErrorCalcYaw < 0)
    {
      *LeADAS_Pct_Rotate = DesiredAutoRotateSpeed(L_ErrorCalcYaw);
    }

    if (L_OdomOffsetX > L_RequestedOffsetX + K_MoveToTagMovementDeadbandX || L_OdomOffsetX < L_RequestedOffsetX - K_MoveToTagMovementDeadbandX)
    {
      *LeADAS_Pct_FwdRev = -C_TagAlignBasePower * (L_OdomOffsetX - L_RequestedOffsetX);
    }
    // else if (L_OdomOffsetX < L_RequestedOffsetX - K_MoveToTagMovementDeadband)
    // {
    //   *LeADAS_Pct_FwdRev = 0.2 * (L_OdomOffsetX - L_RequestedOffsetX);
    // }

    if (L_OdomOffsetY > L_RequestedOffsetY + K_MoveToTagMovementDeadbandY || L_OdomOffsetY < L_RequestedOffsetY - K_MoveToTagMovementDeadbandY)
    {
      *LeADAS_Pct_Strafe = -C_TagAlignBasePower * (L_OdomOffsetY - L_RequestedOffsetY);
    }
    // else if (L_OdomOffsetY < L_RequestedOffsetY - K_MoveToTagMovementDeadband)
    // {
    //   *LeADAS_Pct_Strafe = -0.2 * (L_OdomOffsetY - L_RequestedOffsetY);
    // }

    if (L_OdomOffsetX <= L_RequestedOffsetX + K_MoveToTagMovementDeadbandX && L_OdomOffsetX >= L_RequestedOffsetX - K_MoveToTagMovementDeadbandX)
    {
      *LeADAS_Pct_FwdRev = 0.0;
    }
    if (L_OdomOffsetY <= L_RequestedOffsetY + K_MoveToTagMovementDeadbandY && L_OdomOffsetY >= L_RequestedOffsetY - K_MoveToTagMovementDeadbandY)
    {
      *LeADAS_Pct_Strafe = 0.0;
    }

    if (*LeADAS_Pct_Strafe == 0.0 && *LeADAS_Pct_FwdRev == 0.0)
    {
      LeADAS_b_DM_StateComplete = true;
    }
  // }
  return (LeADAS_b_DM_StateComplete);
}

/******************************************************************************
 * Function:     MoveWithGlobalCoords
 *
 * Description:  TBD
 ******************************************************************************/
bool MoveWithGlobalCoords(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          bool L_OdomCentered,
                          double L_TagYawDegrees,
                          double L_CurrentOdomX,
                          double L_CurrentOdomY,
                          double L_RequestedCoordX,
                          double L_RequestedCoordY)
{
  bool LeADAS_b_DM_StateComplete = false;
  double L_ErrorCalcYaw;
  
  if (L_OdomCentered)
  {

    L_ErrorCalcYaw = 0.18 - L_TagYawDegrees; // -.18 is just what it happened to be idk

    if (L_ErrorCalcYaw > 0 || L_ErrorCalcYaw < 0)
    {
      *LeADAS_Pct_Rotate = DesiredAutoRotateSpeed(L_ErrorCalcYaw);
    }

    if (L_CurrentOdomX > L_RequestedCoordX || L_CurrentOdomX < L_RequestedCoordX)
    {
      *LeADAS_Pct_FwdRev = -C_TagAlignBasePower * (L_CurrentOdomX - L_RequestedCoordX);
    }


    if (L_CurrentOdomY > L_RequestedCoordY + K_MoveToTagMovementDeadbandY || L_CurrentOdomY < L_RequestedCoordY - K_MoveToTagMovementDeadbandY)
    {
      *LeADAS_Pct_Strafe = -C_TagAlignBasePower * (L_CurrentOdomY - L_RequestedCoordY);
    }


    if (L_CurrentOdomX <= L_RequestedCoordX + K_MoveToTagMovementDeadbandX && L_CurrentOdomX >= L_RequestedCoordX - K_MoveToTagMovementDeadbandX)
    {
      *LeADAS_Pct_FwdRev = 0.0;
    }
    if (L_CurrentOdomY <= L_RequestedCoordY + K_MoveToTagMovementDeadbandY && L_CurrentOdomY >= L_RequestedCoordY - K_MoveToTagMovementDeadbandY)
    {
      *LeADAS_Pct_Strafe = 0.0;
    }

    if (*LeADAS_Pct_Strafe == 0.0 && *LeADAS_Pct_FwdRev == 0.0)
    {
      LeADAS_b_DM_StateComplete = true;
    }
  }
  return (LeADAS_b_DM_StateComplete);

}

