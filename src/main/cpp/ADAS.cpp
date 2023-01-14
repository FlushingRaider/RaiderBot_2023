/*
  ADAS.cpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)
  Contains the logic and code used for driver assitance control.  This is meant 
  to serve as a high level controller sending commands/requests to the lower 
  level controls while also tracking and managing the various sytems. This 
  contains manages the following features:

  - Auto upper targeting
    - Turns on camera light, auto centers robot on target, spins the rollers up to the correct speed, disables camera light
  - Auto ball targeting
    - Centers robot on ball, turns on intake roller, drives froward to intake ball, exits
  - Auton Opt 1
    - More info to come

  Changes:
  2022-02-25 -> Beta
  2022-03-12 -> Auto targets, shoots
 */

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "Const.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_BT.hpp"
#include "ADAS_DM.hpp"

/* ADAS control state variables */
T_ADAS_ActiveFeature                            V_ADAS_ActiveFeature = E_ADAS_Disabled;
T_ADAS_ActiveAutonFeature                       V_ADAS_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
frc::SendableChooser<T_ADAS_ActiveAutonFeature> V_ADAS_AutonChooser;
bool                                            V_ADAS_StateComplete = false;
bool                                            V_ADAS_AutonOncePerTrigger = false;
T_ADAS_Auton1                                   V_ADAS_Auton1State;
int                                             V_ADAS_PathNum;

/* ADAS output control variables */
double               V_ADAS_Pct_SD_FwdRev = 0;
double               V_ADAS_Pct_SD_Strafe = 0;
double               V_ADAS_Pct_SD_Rotate = 0;
double               V_ADAS_RPM_BH_Launcher = 0;
double               V_ADAS_Pct_BH_Intake = 0;
double               V_ADAS_Pct_BH_Elevator = 0;
bool                 V_ADAS_CameraUpperLightCmndOn = false;
bool                 V_ADAS_CameraLowerLightCmndOn = false;
bool                 V_ADAS_SD_RobotOriented = false;
bool                 V_ADAS_Vision_RequestedTargeting = false; 
double               V_ADAS_DriveTime = 0;
double               V_ADAS_Deg_TargetAngle = 0;



/******************************************************************************
 * Function:     ADAS_Main_Init
 *
 * Description:  Initialize all applicable ADAS variables at robot init.
 ******************************************************************************/
void ADAS_Main_Init(void)
  {
  std::string_view L_AutonSelectorName = "Auton";
  V_ADAS_AutonChooser.AddOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  V_ADAS_AutonChooser.AddOption("Blind Shot 1", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveAndShootBlind1);
  V_ADAS_AutonChooser.AddOption("Blind Shot 2", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveAndShootBlind2);
  V_ADAS_AutonChooser.AddOption("Auto Shot 2", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveAndShootAuto2);
  V_ADAS_AutonChooser.AddOption("Auto Shot 3", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveAndShootAuto3);
  V_ADAS_AutonChooser.SetDefaultOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  frc::SmartDashboard::PutData(L_AutonSelectorName, &V_ADAS_AutonChooser);
  }


/******************************************************************************
 * Function:     ADAS_DetermineMode
 *
 * Description:  Ping the driver station to see what the desired auton routine 
 *               should be.
 ******************************************************************************/
void ADAS_DetermineMode(void)
  {
  V_ADAS_DriverRequestedAutonFeature = V_ADAS_AutonChooser.GetSelected();
  frc::SmartDashboard::PutNumber("Requested Auton",  float(V_ADAS_DriverRequestedAutonFeature));
  }


/******************************************************************************
 * Function:     ADAS_Main_Reset
 *
 * Description:  Reset all applicable ADAS variables.
 ******************************************************************************/
void ADAS_Main_Reset(void)
  {
  V_ADAS_ActiveFeature = E_ADAS_Disabled;
  V_ADAS_Pct_SD_FwdRev = 0;
  V_ADAS_Pct_SD_Strafe = 0;
  V_ADAS_Pct_SD_Rotate = 0;
  V_ADAS_RPM_BH_Launcher = 0;
  V_ADAS_Pct_BH_Intake = 0;
  V_ADAS_Pct_BH_Elevator = 0;
  V_ADAS_CameraUpperLightCmndOn = false;
  V_ADAS_CameraLowerLightCmndOn = false;
  V_ADAS_SD_RobotOriented = false;
  V_ADAS_Vision_RequestedTargeting = false;
  V_ADAS_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
  V_ADAS_StateComplete = false;
  V_ADAS_AutonOncePerTrigger = false;
  V_ADAS_DriveTime = 0;
  V_ADAS_Deg_TargetAngle = 0;
  
  /* Trigger the resets for all of the sub tasks/functions as well: */
  ADAS_UT_Reset();
  ADAS_BT_Reset();
  ADAS_DM_Reset();
  }


/******************************************************************************
 * Function:     ADAS_ControlMain
 *
 * Description:  Main calling function for the ADAS (advanced driver assistance 
 *               system)control when robot is active. This will call and manage 
 *               the various ADAS features. 
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_ControlMain(double               *L_Pct_FwdRev,
                                      double               *L_Pct_Strafe,
                                      double               *L_Pct_Rotate,
                                      double               *L_RPM_Launcher,
                                      double               *L_Pct_Intake,
                                      double               *L_Pct_Elevator,
                                      bool                 *L_CameraUpperLightCmndOn,
                                      bool                 *L_CameraLowerLightCmndOn,
                                      bool                 *L_SD_RobotOriented,
                                      bool                 *L_VisionTargetingRequest,
                                      bool                  L_Driver1_JoystickActive,
                                      bool                  L_Driver_stops_shooter,
                                      bool                  L_Driver_SwerveGoalAutoCenter,
                                      bool                  L_Driver_AutoIntake,
                                      double                L_Deg_GyroAngleDeg,
                                      double                L_L_X_FieldPos,
                                      double                L_L_Y_FieldPos,
                                      bool                  L_VisionTopTargetAquired,
                                      double                L_TopTargetYawDegrees,
                                      double                L_VisionTopTargetDistanceMeters,
                                      bool                  L_VisionBottomTargetAquired,
                                      double                L_VisionBottomYaw,
                                      double                L_VisionBottomTargetDistanceMeters,
                                      T_RobotState          L_RobotState,
                                      double                L_LauncherRPM_Measured,
                                      bool                  L_BallDetectedUpper,
                                      bool                  L_BallDetectedLower,
                                      bool                  L_DriverRequestElevatorUp,
                                      bool                  L_DriverRequestElevatorDwn,
                                      bool                  L_DriverRequestIntake,
                                      T_ADAS_ActiveFeature  L_ADAS_ActiveFeature)
  {
  T_ADAS_ActiveFeature L_ADAS_ActiveFeaturePrev = L_ADAS_ActiveFeature;

  /* First, let's determine what we are going to do: */
  if (L_RobotState == E_Teleop)
    {
    /* Enable criteria goes here: */
    if (L_Driver_SwerveGoalAutoCenter == true)
      {
      L_ADAS_ActiveFeature = E_ADAS_UT_AutoUpperTarget;
      }
    else if (L_Driver_AutoIntake == true)
      {
      L_ADAS_ActiveFeature = E_ADAS_BT_AutoBallTarget;
      }
  
    /* Abort criteria goes here: */
    if ((L_Driver1_JoystickActive == true) || (L_Driver_stops_shooter == true) || (V_ADAS_StateComplete == true))
      {
      /* Abort criteria goes here. */
      L_ADAS_ActiveFeature = E_ADAS_Disabled;
      V_ADAS_StateComplete = false;
      }
    }
  else if (L_RobotState == E_Auton)
    {
    if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind1)
      {
      if ((L_ADAS_ActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_BlindLaunch;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_BlindLaunch) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_DriveStraight;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_DriveStraight) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
        }
      }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind2)
      {
      if ((L_ADAS_ActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_ReverseAndIntake;
        V_ADAS_DriveTime = K_ADAS_DM_DriveTimeLong;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_ReverseAndIntake) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_Rotate180;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_Rotate180) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_BlindLaunch;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_BlindLaunch) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
        }
      }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootAuto2)
      {
      if ((L_ADAS_ActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_ReverseAndIntake;
        V_ADAS_DriveTime = K_ADAS_DM_DriveTimeShort;
        V_ADAS_DM_InitGyroAngle = L_Deg_GyroAngleDeg;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_ReverseAndIntake) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_BT_AutoBallTarget;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_BT_AutoBallTarget) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_RotateFieldOriented;
        V_ADAS_Deg_TargetAngle = L_Deg_GyroAngleDeg + 180;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_DM_RotateFieldOriented) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_UT_AutoUpperTarget;
        }
      else if ((L_ADAS_ActiveFeature == E_ADAS_UT_AutoUpperTarget) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
        }
      }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootAuto3)
      {
      if ((L_ADAS_ActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 1;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_1;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_1) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_BT_AutoBallTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_BT_2;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_BT_2) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 2;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_3;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_3) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_UT_AutoUpperTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_UT_4;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_UT_4) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 3;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_5;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_5) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_BT_AutoBallTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_BT_6;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_BT_6) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_DM_RotateFieldOriented;
        V_ADAS_Deg_TargetAngle = L_Deg_GyroAngleDeg + 180;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_Rotate_7;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_Rotate_7) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_UT_AutoUpperTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_UT_8;
        }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_UT_8) &&
               (V_ADAS_StateComplete == true))
        {
        L_ADAS_ActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
        }
      }
    else
      {
      /* No auton requested. */
      L_ADAS_ActiveFeature = E_ADAS_Disabled;
      }
    }
  else
    {
    L_ADAS_ActiveFeature = E_ADAS_Disabled;
    }

  if (L_ADAS_ActiveFeature == E_ADAS_Disabled)
    {
    /* Hmm, there was a transition, let's go ahead and reset all of the variables before we start: */
    ADAS_UT_Reset();
    ADAS_BT_Reset();
    ADAS_DM_Reset();
    V_ADAS_StateComplete = false;
    }

  switch (L_ADAS_ActiveFeature)
    {
      case E_ADAS_UT_AutoUpperTarget:
          V_ADAS_StateComplete = ADAS_UT_Main(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraUpperLightCmndOn,
                                              L_CameraLowerLightCmndOn,
                                              L_SD_RobotOriented,
                                              L_VisionTargetingRequest,
                                              L_VisionTopTargetAquired,
                                              L_TopTargetYawDegrees,
                                              L_VisionTopTargetDistanceMeters,
                                              L_RobotState,
                                              L_LauncherRPM_Measured,
                                              L_BallDetectedUpper,
                                              L_DriverRequestElevatorUp,
                                              L_DriverRequestElevatorDwn,
                                              L_DriverRequestIntake);
      break;
      case E_ADAS_BT_AutoBallTarget:
          V_ADAS_StateComplete = ADAS_BT_Main(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraUpperLightCmndOn,
                                              L_CameraLowerLightCmndOn,
                                              L_SD_RobotOriented,
                                              L_VisionTargetingRequest,
                                              L_VisionBottomTargetAquired,
                                              L_VisionBottomYaw,
                                              L_VisionBottomTargetDistanceMeters,
                                              L_RobotState,
                                              L_BallDetectedUpper,
                                              L_BallDetectedLower);
      break;
      case E_ADAS_DM_BlindLaunch:
          V_ADAS_StateComplete = ADAS_DM_BlindShot(L_Pct_FwdRev,
                                                      L_Pct_Strafe,
                                                      L_Pct_Rotate,
                                                      L_RPM_Launcher,
                                                      L_Pct_Intake,
                                                      L_Pct_Elevator,
                                                      L_CameraUpperLightCmndOn,
                                                      L_CameraLowerLightCmndOn,
                                                      L_SD_RobotOriented);
      break;
      case E_ADAS_DM_DriveStraight:
          V_ADAS_StateComplete = ADAS_DM_DriveStraight(L_Pct_FwdRev,
                                                          L_Pct_Strafe,
                                                          L_Pct_Rotate,
                                                          L_RPM_Launcher,
                                                          L_Pct_Intake,
                                                          L_Pct_Elevator,
                                                          L_CameraUpperLightCmndOn,
                                                          L_CameraLowerLightCmndOn,
                                                          L_SD_RobotOriented);
      break;
      case E_ADAS_DM_ReverseAndIntake:
          V_ADAS_StateComplete = ADAS_DM_ReverseAndIntake(L_Pct_FwdRev,
                                                             L_Pct_Strafe,
                                                             L_Pct_Rotate,
                                                             L_RPM_Launcher,
                                                             L_Pct_Intake,
                                                             L_Pct_Elevator,
                                                             L_CameraUpperLightCmndOn,
                                                             L_CameraLowerLightCmndOn,
                                                             L_SD_RobotOriented,
                                                             V_ADAS_DriveTime);
      break;
      case E_ADAS_DM_Rotate180:
          V_ADAS_StateComplete = ADAS_DM_Rotate180(L_Pct_FwdRev,
                                                      L_Pct_Strafe,
                                                      L_Pct_Rotate,
                                                      L_RPM_Launcher,
                                                      L_Pct_Intake,
                                                      L_Pct_Elevator,
                                                      L_CameraUpperLightCmndOn,
                                                      L_CameraLowerLightCmndOn,
                                                      L_SD_RobotOriented,
                                                      L_Deg_GyroAngleDeg);
      break;
      case E_ADAS_DM_RotateFieldOriented:
          V_ADAS_StateComplete = ADAS_DM_FieldOrientRotate(L_Pct_FwdRev,
                                                      L_Pct_Strafe,
                                                      L_Pct_Rotate,
                                                      L_RPM_Launcher,
                                                      L_Pct_Intake,
                                                      L_Pct_Elevator,
                                                      L_CameraUpperLightCmndOn,
                                                      L_CameraLowerLightCmndOn,
                                                      L_SD_RobotOriented,
                                                      L_Deg_GyroAngleDeg,
                                                      V_ADAS_Deg_TargetAngle);
      break;
      case E_ADAS_DM_PathFollower:
          V_ADAS_StateComplete = ADAS_DM_PathFollower(L_Pct_FwdRev,
                                                      L_Pct_Strafe,
                                                      L_Pct_Rotate,
                                                      L_RPM_Launcher,
                                                      L_Pct_Intake,
                                                      L_Pct_Elevator,
                                                      L_CameraUpperLightCmndOn,
                                                      L_CameraLowerLightCmndOn,
                                                      L_SD_RobotOriented,
                                                      L_L_X_FieldPos,
                                                      L_L_Y_FieldPos,
                                                      L_Deg_GyroAngleDeg,
                                                      V_ADAS_PathNum);
      break;
      case E_ADAS_Disabled:
      default:
          *L_Pct_FwdRev = 0;
          *L_Pct_Strafe = 0;
          *L_Pct_Rotate = 0;
          *L_RPM_Launcher = 0;
          *L_Pct_Intake = 0;
          *L_Pct_Elevator = 0;
          *L_CameraUpperLightCmndOn = false;
          *L_CameraLowerLightCmndOn = false;
          *L_VisionTargetingRequest = false;
      break;
    }

  return (L_ADAS_ActiveFeature);
  }