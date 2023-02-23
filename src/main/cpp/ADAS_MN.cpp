/*
  ADAS_MN.cpp

  Created on: Feb 21, 2023
  Author: Jay L

  ADAS (Advanced Driver-Assistance Systems) Upper Targeting
  Contains the logic and code used for the upper targeting control:
    - Schedules states in a non-linear state machine
  Changes:
  2023-02-21 -> Alpha
 */


#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Const.hpp"
#include <frc/DriverStation.h>
#include "ADAS_MN.hpp"
// bool    LeCONT_b_Driver2ButtonA,
// bool    LeCONT_b_Driver2ButtonB,
// bool    LeCONT_b_Driver2ButtonRB,
// bool    LeCONT_b_Driver2ButtonLB,
// bool    LeCONT_b_Driver2ButtonStart,
// bool    LeCONT_b_Driver2ButtonX,
// bool    LeCONT_b_Driver2ButtonY,
// double  LeCONT_Pct_Driver2LeftAxisY,
// double  LeCONT_Pct_Driver2RightAxisX,
// int     LeCONT_Deg_Driver2POV,
// bool    LeCONT_b_Driver2ButtonBack,
// double  LeCont_Pct_Driver2AxisRB,
// double  LeCont_Pct_Driver2AxisLB


TeMAN_ManipulatorStates VeADAS_E_VeADAS_E_ScheduledState = E_ADAS_MN_Disabled; //State Scheduled in relation to driver input. Used for non-linear state machines
double V_ADAS_MN_DebounceTime = 0;
double V_ADAS_MN_RotateErrorPrev = 0;
double V_ADAS_MN_LauncherSpeedPrev = 0;
bool V_ADAS_MN_TargetAquiredPrev = false;

/* Configuration cals: */
double KV_ADAS_MN_LostTargetGx;
double KV_ADAS_MN_NoTargetError;
double KV_ADAS_MN_DebounceTime;
double KV_ADAS_MN_RotateDeadbandAngle;
double KV_ADAS_MN_TargetVisionAngle;

/******************************************************************************
 * Function:     ADAS_MN_ConfigsInit
 *
 * Description:  Contains the configurations for the UT.
 ******************************************************************************/
void ADAS_MN_ConfigsInit()
{
  // set coefficients

  // KV_ADAS_MN_LightDelayTIme = K_ADAS_MN_LightDelayTime;
  // KV_ADAS_MN_LostTargetGx = K_ADAS_MN_LostTargetGx;
  // KV_ADAS_MN_NoTargetError = K_ADAS_MN_NoTargetError;
  // KV_ADAS_MN_DebounceTime = K_ADAS_MN_DebounceTime;
  // KV_ADAS_MN_RotateDeadbandAngle = K_ADAS_MN_RotateDeadbandAngle;
  // KV_ADAS_MN_TargetVisionAngle = K_ADAS_MN_TargetVisionAngle;

#ifdef ADAS_MN_Test
  // display coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_LightDelayTIme", KV_ADAS_MN_LightDelayTIme);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_LostTargetGx", KV_ADAS_MN_LostTargetGx);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_NoTargetError", KV_ADAS_MN_NoTargetError);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_DebounceTime", KV_ADAS_MN_DebounceTime);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_AllowedLauncherError", KV_ADAS_MN_AllowedLauncherError);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_AllowedLauncherTime", KV_ADAS_MN_AllowedLauncherTime);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_RotateDeadbandAngle", KV_ADAS_MN_RotateDeadbandAngle);
  frc::SmartDashboard::PutNumber("KV_ADAS_MN_TargetVisionAngle", KV_ADAS_MN_TargetVisionAngle);
#endif
}

/******************************************************************************
 * Function:     ADAS_MN_ConfigsCal
 *
 * Description:  Contains the motor configurations for the Manipulator motors.  This
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ADAS_MN_ConfigsCal()
{
// read coefficients from SmartDashboard
#ifdef ADAS_MN_Test
  KV_ADAS_MN_LightDelayTIme = frc::SmartDashboard::GetNumber("KV_ADAS_MN_LightDelayTIme", KV_ADAS_MN_LightDelayTIme);
  KV_ADAS_MN_LostTargetGx = frc::SmartDashboard::GetNumber("KV_ADAS_MN_LostTargetGx", KV_ADAS_MN_LostTargetGx);
  KV_ADAS_MN_NoTargetError = frc::SmartDashboard::GetNumber("KV_ADAS_MN_NoTargetError", KV_ADAS_MN_NoTargetError);
  KV_ADAS_MN_DebounceTime = frc::SmartDashboard::GetNumber("KV_ADAS_MN_DebounceTime", KV_ADAS_MN_DebounceTime);
  KV_ADAS_MN_AllowedLauncherError = frc::SmartDashboard::GetNumber("KV_ADAS_MN_AllowedLauncherError", KV_ADAS_MN_AllowedLauncherError);
  KV_ADAS_MN_AllowedLauncherTime = frc::SmartDashboard::GetNumber("KV_ADAS_MN_AllowedLauncherTime", KV_ADAS_MN_AllowedLauncherTime);
  KV_ADAS_MN_RotateDeadbandAngle = frc::SmartDashboard::GetNumber("KV_ADAS_MN_RotateDeadbandAngle", KV_ADAS_MN_RotateDeadbandAngle);
  KV_ADAS_MN_TargetVisionAngle = frc::SmartDashboard::GetNumber("KV_ADAS_MN_TargetVisionAngle", KV_ADAS_MN_TargetVisionAngle);
#endif
}

/******************************************************************************
 * Function:     ADAS_MN_Reset
 *
 * Description:  Reset all applicable MN variables.
 ******************************************************************************/
void ADAS_MN_Reset(void)
{
  TeMAN_ManipulatorStates VeADAS_E_VeADAS_E_ScheduledState = E_ADAS_MN_Disabled;
  V_ADAS_MN_DebounceTime = 0;
  V_ADAS_MN_RotateErrorPrev = 0;
  V_ADAS_MN_LauncherSpeedPrev = 0;
  V_ADAS_MN_TargetAquiredPrev = false;
  #ifdef unfinished
  VeADAS_E_ScheduledState = E_Rest
  #endif
}

#ifdef unfinished
/******************************************************************************
 * Function:    VeADAS_E_ScheduledState
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled states based on current
 ******************************************************************************/
 T_ADAS_MN_ManipulatorStates VeADAS_E_ScheduledState (double *L_Pct_FwdRev,
                                            double *L_Pct_Strafe,
                                            double *L_Pct_Rotate,
                                            double *L_RPM_Launcher,
                                            double *L_Pct_Intake,
                                            double *L_Pct_Elevator,
                                            bool *L_SD_RobotOriented,
                                            bool *L_VisionTargetingRequest)
{
  T_ADAS_MN_UpperTarget L_ADAS_MN_State = 

  /* Next, set all other values to off as we are just wanting to command the light on: */
  *L_CameraLowerLightCmndOn = false;
  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  *L_Pct_Rotate = 0;
  *L_RPM_Launcher = 0;
  *L_Pct_Intake = 0;
  *L_Pct_Elevator = 0;
  /* Indicate that the light is on and we can proceed: */
  /* Start incremeting a debounce time.  We want to give a bit of time for the camera to have light: */
  V_ADAS_MN_DebounceTime += C_ExeTime;

  if (V_ADAS_MN_DebounceTime >= KV_ADAS_MN_LightDelayTIme)
  {
    L_ADAS_MN_State = E_ADAS_MN_AutoCenter;
    V_ADAS_MN_DebounceTime = 0;
  }

  return (L_ADAS_MN_State);

  if (VsCONT_s_DriverInput.b_IntakeOut == true)
  {VeADAS_E_ScheduledState = E_Swiper};
  else if (VsCONT_s_DriverInput.b_DrivingPosition == true)
  {VeADAS_E_ScheduledState = E_DrivingState};
     else if (VsCONT_s_DriverInput.b_DropGamePiece == true)
     {VeADAS_E_ScheduledState = E_DroppingTheLoot};
       else if (VsCONT_s_DriverInput.b_IntakeRollers == true)
       {VeADAS_E_ScheduledState = E_TradeOff};
         else if (VsCONT_s_DriverInput.b_UpperSore == true)
         {VeADAS_E_ScheduledState = E_PositioningState};
         else if (VsCONT_s_DriverInput.b_LowerScore == true)
         {VeADAS_E_ScheduledState = E_PositioningState};
           else if (VsCONT_s_DriverInput.b_IntakeArmOut = true)
           {VeADAS_E_ScheduledState = E_PositioningState};
           else if (VsCONT_s_DriverInput.b_IntakeArmIn = true)
           {VeADAS_E_ScheduledState = E_DrivingState};
}
#endif

/******************************************************************************
 * Function:     ADAS_MN_MoveToTag
 * Author: Carson
 * Description: Moves to the closest tag scanned
 ******************************************************************************/
TeMAN_ManipulatorStates ADAS_MN_MoveToTag(double *L_Pct_FwdRev,
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

  TeMAN_ManipulatorStates L_ADAS_MN_State = E_ADAS_MN_MoveToTag;

  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  /* Next, let's set all the other items we aren't trying to control to off: */
  double L_ChosenX = 0;
  double L_ChosenY = 0;
  double L_ErrorCalcYaw = 0;
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

  // double L_Tag1YError;
  // double L_Tag2YError;
  // double L_Tag3YError;

  V_ADAS_MN_DebounceTime += C_ExeTime;
  if (L_OdomCentered) // don't do any of this if we haven't centered our Odometry based on the tag
  {
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
    if (V_ADAS_MN_DebounceTime <= KV_ADAS_MN_DebounceTime) // make sure we're still in the time we've given ourselves
    {
      if (L_ErrorCalcYaw > 0 || L_ErrorCalcYaw < 0)
      {
        *L_Pct_Rotate = DesiredAutoRotateSpeed(L_ErrorCalcYaw);
      }
      if (L_OdometryX < L_ChosenX + K_MoveToTagMovementDeadband || L_OdometryX < L_ChosenX - K_MoveToTagMovementDeadband)
      {
        *L_Pct_FwdRev = 0.2;
      }
      else if (L_OdometryX > L_ChosenX + K_MoveToTagMovementDeadband || L_OdometryX > L_ChosenX - K_MoveToTagMovementDeadband)
      {
        *L_Pct_FwdRev = -0.2;
      }
      else
      {
        *L_Pct_FwdRev = 0.0;
      }
      if (L_OdometryY < L_ChosenY + K_MoveToTagMovementDeadband || L_OdometryY < L_ChosenY - K_MoveToTagMovementDeadband)
      {
        *L_Pct_Strafe = 0.2;
      }
      else if (L_OdometryY > L_ChosenY + K_MoveToTagMovementDeadband || L_OdometryY > L_ChosenY - K_MoveToTagMovementDeadband)
      {
        *L_Pct_Strafe = -0.2;
      }
      else
      {
        *L_Pct_Strafe = 0.0;
      }
    }
    else if (V_ADAS_MN_DebounceTime >= 0.3)
    {
      V_ADAS_MN_DebounceTime = 0;
      *L_Pct_FwdRev = 0;
      *L_Pct_Strafe = 0;
    }
  }
  return (L_ADAS_MN_State);
}

/******************************************************************************
 * Function:     ADAS_MN_Main
 *
 * Description:  Manages and controls the upper targeting (UT) and spinning up of
 *               the launcher.
 ******************************************************************************/
bool ADAS_MN_Main(double *L_Pct_FwdRev,
                  double *L_Pct_Strafe,
                  double *L_Pct_Rotate,
                  double *L_Pct_Intake,
                  bool *L_VisionTargetingRequest,
                  bool L_VisionTopTargetAquired,
                  T_RobotState L_RobotState,
                  int L_TagID,
                  bool L_OdomCentered,
                  double L_OdometryX,
                  double L_OdometryY,
                  double L_TagYawDegrees,
                  frc::DriverStation::Alliance LeLC_e_AllianceColor,
                  bool L_CubeAlignCmd,
                  bool L_ConeAlignCmd)
{
  bool L_ADAS_MN_Complete = false;

  switch (VeADAS_E_VeADAS_E_ScheduledState)
  {
  case E_ADAS_MN_Disabled:
  break;
  #ifdef unused
  case E_ADAS_MN_CameraLightOn:

    V_ADAS_MN_State = ADAS_MN_CameraLightOn(L_Pct_FwdRev,
                                            L_Pct_Strafe,
                                            L_Pct_Rotate,
                                            L_RPM_Launcher,
                                            L_Pct_Intake,
                                            L_Pct_Elevator,
                                            L_CameraUpperLightCmndOn,
                                            L_CameraLowerLightCmndOn,
                                            L_SD_RobotOriented,
                                            L_VisionTargetingRequest);
    break;
#endif
  case E_ADAS_MN_MoveToTag:
    VeADAS_E_VeADAS_E_ScheduledState = ADAS_MN_MoveToTag(L_Pct_FwdRev,
                                        L_Pct_Strafe,
                                        L_Pct_Rotate,
                                        L_OdomCentered,
                                        L_TagID,
                                        L_OdometryX,
                                        L_OdometryY,
                                        L_VisionTargetingRequest,
                                        L_VisionTopTargetAquired,
                                        L_TagYawDegrees,
                                        LeLC_e_AllianceColor,
                                        L_CubeAlignCmd,
                                        L_ConeAlignCmd);
    break;
  }

  if (VeADAS_E_VeADAS_E_ScheduledState == E_ADAS_MN_Disabled)
  {
    ADAS_MN_Reset();
    L_ADAS_MN_Complete = true;
  }

  return (L_ADAS_MN_Complete);
}