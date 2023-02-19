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
#include <frc/DriverStation.h>

#include "Const.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_BT.hpp"
#include "ADAS_DM.hpp"
#include "ADAS_ARM.hpp"

/* ADAS control state variables */
T_ADAS_ActiveFeature V_ADAS_ActiveFeature = E_ADAS_Disabled;
T_ADAS_ActiveAutonFeature V_ADAS_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
frc::SendableChooser<T_ADAS_ActiveAutonFeature> V_ADAS_AutonChooser;
bool V_ADAS_StateComplete = false;
bool V_ADAS_AutonOncePerTrigger = false;
T_ADAS_Auton1 V_ADAS_Auton1State;
int V_ADAS_PathNum;
std::string V_ADAS_Auto_PathName;

/* ADAS output control variables */
double V_ADAS_Pct_SD_FwdRev = 0;
double V_ADAS_Pct_SD_Strafe = 0;
double V_ADAS_Pct_SD_Rotate = 0;
#ifdef unused
double V_ADAS_RPM_BH_Launcher = 0;
#endif
double V_ADAS_Pct_BH_Intake = 0;
double V_ADAS_Pct_BH_Elevator = 0;
bool V_ADAS_CameraUpperLightCmndOn = false;
bool V_ADAS_CameraLowerLightCmndOn = false;
bool V_ADAS_SD_RobotOriented = false;
bool V_ADAS_Vision_RequestedTargeting = false;
double V_ADAS_DriveTime = 0;
double V_ADAS_Deg_TargetAngle = 0;

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
  frc::SmartDashboard::PutNumber("Requested Auton", float(V_ADAS_DriverRequestedAutonFeature));
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
#ifdef unused
  V_ADAS_RPM_BH_Launcher = 0;
#endif
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

#ifdef unused
  ADAS_BT_Reset();
#endif

  ADAS_DM_Reset();
}

/******************************************************************************
 * Function:     ADAS_ControlMain
 *
 * Description:  Main calling function for the ADAS (advanced driver assistance
 *               system)control when robot is active. This will call and manage
 *               the various ADAS features.
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_ControlMain(double *L_Pct_FwdRev,
                                      double *L_Pct_Strafe,
                                      double *L_Pct_Rotate,
                                      double *L_Pct_Intake,
                                      bool *L_SD_RobotOriented,
                                      bool *L_VisionTargetingRequest,
                                      bool L_Driver1_JoystickActive,
                                      bool L_Driver_SwerveGoalAutoCenter,
                                      double L_Deg_GyroAngleDeg,
                                      double L_L_X_FieldPos,
                                      double L_L_Y_FieldPos,
                                      bool L_VisionTopTargetAquired,
                                      T_RobotState L_RobotState,
                                      T_ADAS_ActiveFeature LeLC_e_ADASActiveFeature,
                                      int L_TagID,
                                      bool L_OdomCentered,
                                      double L_TagYawDegrees,
                                      frc::DriverStation::Alliance LeLC_e_AllianceColor)
{

  /* First, let's determine what we are going to do: */
  if (L_RobotState == E_Teleop)
  {
    /* Enable criteria goes here: */
    if (L_Driver_SwerveGoalAutoCenter == true)
    {
      LeLC_e_ADASActiveFeature = E_ADAS_UT_AutoUpperTarget;
    }

    /* Abort criteria goes here: */
    if ((L_Driver1_JoystickActive == true) || (V_ADAS_StateComplete == true))
    {
      /* Abort criteria goes here. */
      LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
      V_ADAS_StateComplete = false;
    }
  }
<<<<<<< HEAD
  else if (L_RobotState == E_Auton) // Are we in auton state?
  {
    if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind1) // Check what auton feature we want
=======
  else if (L_RobotState == E_Auton)
  {
    if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind1)
>>>>>>> 3a98709516d8b7b7691637cff08000236949ab06
    {
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_BlindLaunch;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_BlindLaunch) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_DriveStraight;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_DriveStraight) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind2)
    {
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_ReverseAndIntake;
        V_ADAS_DriveTime = K_ADAS_DM_DriveTimeLong;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_ReverseAndIntake) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_Rotate180;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_Rotate180) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_BlindLaunch;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_BlindLaunch) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootAuto2)
    {
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_ReverseAndIntake;
        V_ADAS_DriveTime = K_ADAS_DM_DriveTimeShort;
        V_ADAS_DM_InitGyroAngle = L_Deg_GyroAngleDeg;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_ReverseAndIntake) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_BT_AutoBallTarget;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_BT_AutoBallTarget) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_RotateFieldOriented;
        V_ADAS_Deg_TargetAngle = L_Deg_GyroAngleDeg + 180;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_DM_RotateFieldOriented) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_UT_AutoUpperTarget;
      }
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_UT_AutoUpperTarget) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootAuto3)
    {
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) &&
          (V_ADAS_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 1;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_1;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_1) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_BT_AutoBallTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_BT_2;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_BT_2) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 2;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_3;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_3) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_UT_AutoUpperTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_UT_4;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_UT_4) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 3;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_5;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_5) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_BT_AutoBallTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_BT_6;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_BT_6) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_RotateFieldOriented;
        V_ADAS_Deg_TargetAngle = L_Deg_GyroAngleDeg + 180;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_Rotate_7;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_Rotate_7) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_UT_AutoUpperTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_UT_8;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_UT_8) &&
               (V_ADAS_StateComplete == true))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
        V_ADAS_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDeployCone) // Auton code for deplying Cones
    {
      // Step 1 - Place Cone
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) && (V_ADAS_StateComplete == false) && (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_AutonDeployCone;
      }
      // Step 2
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_AutonDeployCone) && (V_ADAS_StateComplete == true))
      {
        // LeLC_e_ADASActiveFeature = AUTONCOMMAND;
      }
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDeployCube) // Auton code for deplying Cubes
    {
      // Step 1 - Place Cube
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) && (V_ADAS_StateComplete == false) && (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_AutonDeployCube;
      }
      // Step 2
      else if ((LeLC_e_ADASActiveFeature == E_ADAS_AutonDeployCube) && (V_ADAS_StateComplete == true))
      {
        // LeLC_e_ADASActiveFeature = AUTONCOMMAND;
      }
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonDrivePath) // Path driver
    {
<<<<<<< HEAD
      if ((LeLC_e_ADASActiveFeature == E_ADAS_Disabled) && (V_ADAS_StateComplete == false) && (V_ADAS_AutonOncePerTrigger == false))
      {
        LeLC_e_ADASActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 0; //Zero to use path auto load
        V_ADAS_Auto_PathName = "test1"; //load test1.wpilib.json in deploy/paths/
      }
=======
>>>>>>> 3a98709516d8b7b7691637cff08000236949ab06
    }
    else if (V_ADAS_DriverRequestedAutonFeature == E_ADAS_AutonRotate)
    {
    }
<<<<<<< HEAD
    else // No auton selected
=======
    else
>>>>>>> 3a98709516d8b7b7691637cff08000236949ab06
    {
      /* No auton requested. */
      LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
    }
  }
  else
  {
    LeLC_e_ADASActiveFeature = E_ADAS_Disabled;
  }

  if (LeLC_e_ADASActiveFeature == E_ADAS_Disabled)
  {
    /* Hmm, there was a transition, let's go ahead and reset all of the variables before we start: */
    ADAS_UT_Reset();
#ifdef unused
    ADAS_BT_Reset();
#endif
    ADAS_DM_Reset();
    V_ADAS_StateComplete = false;
  }

  switch (LeLC_e_ADASActiveFeature)
  {
  case E_ADAS_UT_AutoUpperTarget:
    V_ADAS_StateComplete = ADAS_UT_Main(L_Pct_FwdRev,
                                        L_Pct_Strafe,
                                        L_Pct_Rotate,
                                        L_Pct_Intake,
                                        L_VisionTargetingRequest,
                                        L_VisionTopTargetAquired,
                                        L_RobotState,
                                        L_OdomCentered,
                                        L_TagID,
                                        L_L_X_FieldPos,
                                        L_L_Y_FieldPos,
                                        L_TagYawDegrees,
                                        LeLC_e_AllianceColor);
    break;
  case E_ADAS_BT_AutoBallTarget:
#ifdef unused
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
#endif
  case E_ADAS_DM_BlindLaunch:
#ifdef unused
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
#endif
  case E_ADAS_DM_DriveStraight:
#ifdef unused
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
#endif
  case E_ADAS_DM_ReverseAndIntake:
#ifdef unused
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
#endif
  case E_ADAS_DM_Rotate180:
#ifdef unused
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
#endif
  case E_ADAS_DM_RotateFieldOriented:
#ifdef unused
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
#endif
  case E_ADAS_DM_PathFollower:
#ifdef unused
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
                                                V_ADAS_PathNum,
                                                V_ADAS_Auto_PathName);
    break;
#endif
  case E_ADAS_Disabled:
  default:
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *L_Pct_Intake = 0;
#ifdef unused
    *L_CameraUpperLightCmndOn = false;
    *L_CameraLowerLightCmndOn = false;
    *L_RPM_Launcher = 0;
    *L_Pct_Elevator = 0;
#endif
    *L_VisionTargetingRequest = false;
    break;
  }

  return (LeLC_e_ADASActiveFeature);
}