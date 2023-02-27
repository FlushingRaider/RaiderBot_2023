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
#include "ADAS_BT.hpp"
#include "ADAS_DM.hpp"
#include "ADAS_ARM.hpp"
#include "ADAS_MN.hpp"
#include "Driver_inputs.hpp"
#include "Gyro.hpp"

/* ADAS control state variables */
T_ADAS_ActiveFeature V_ADAS_ActiveFeature = E_ADAS_Disabled;
T_ADAS_ActiveAutonFeature VeADAS_e_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
frc::SendableChooser<T_ADAS_ActiveAutonFeature> V_ADAS_AutonChooser;
bool VeADAS_b_StateComplete = false;
bool V_ADAS_AutonOncePerTrigger = false;
T_ADAS_Auton1 V_ADAS_Auton1State;
int V_ADAS_PathNum;
std::string V_ADAS_Auto_PathName;

/* ADAS output control variables */
double V_ADAS_Pct_SD_FwdRev = 0;
double V_ADAS_Pct_SD_Strafe = 0;
double V_ADAS_Pct_SD_Rotate = 0;

TeADAS_Controls VsADAS_h_ActuatorCmnd;

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
bool VeADAS_b_X_Mode = false;

bool toggle;

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
  V_ADAS_AutonChooser.AddOption("Test Path", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath1);
  V_ADAS_AutonChooser.SetDefaultOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  frc::SmartDashboard::PutData(L_AutonSelectorName, &V_ADAS_AutonChooser);
  frc::SmartDashboard::PutBoolean("movetotag", toggle);

}

/******************************************************************************
 * Function:     ADAS_DetermineMode
 *
 * Description:  Ping the driver station to see what the desired auton routine
 *               should be.
 ******************************************************************************/
void ADAS_DetermineMode(void)
{
  VeADAS_e_DriverRequestedAutonFeature = V_ADAS_AutonChooser.GetSelected();
  frc::SmartDashboard::PutNumber("Requested Auton", float(VeADAS_e_DriverRequestedAutonFeature));
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

  VsADAS_h_ActuatorCmnd.b_MAN_DropObject = false;
  VsADAS_h_ActuatorCmnd.b_SD_RobotOriented = false;
  VsADAS_h_ActuatorCmnd.e_MAN_State = E_MAN_Driving;
  VsADAS_h_ActuatorCmnd.Pct_SD_FwdRev = 0.0;
  VsADAS_h_ActuatorCmnd.Pct_SD_Rotate = 0.0;
  VsADAS_h_ActuatorCmnd.Pct_SD_Strafe = 0.0;

  V_ADAS_Pct_BH_Intake = 0;
  V_ADAS_Pct_BH_Elevator = 0;
  V_ADAS_CameraUpperLightCmndOn = false;
  V_ADAS_CameraLowerLightCmndOn = false;
  V_ADAS_SD_RobotOriented = false;
  V_ADAS_Vision_RequestedTargeting = false;
  VeADAS_e_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
  VeADAS_b_StateComplete = false;
  V_ADAS_AutonOncePerTrigger = false;
  V_ADAS_DriveTime = 0;
  V_ADAS_Deg_TargetAngle = 0;

  /* Trigger the resets for all of the sub tasks/functions as well: */

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
                                      bool *LeADAS_b_X_Mode,
                                      bool *L_VisionTargetingRequest,
                                      bool LeADAS_b_Driver1_JoystickActive,
                                      bool L_Driver_SwerveGoalAutoCenter,
                                      double L_Deg_GyroAngleDeg,
                                      double L_L_X_FieldPos,
                                      double L_L_Y_FieldPos,
                                      bool L_VisionTopTargetAquired,
                                      T_RobotState LeADAS_e_RobotState,
                                      T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                                      int L_TagID,
                                      bool L_OdomCentered,
                                      double L_TagYawDegrees,
                                      frc::DriverStation::Alliance LeLC_e_AllianceColor,
                                      bool L_CubeAlignCmd,
                                      bool L_ConeAlignCmd)
{
  bool LeADAS_b_State1Complete = false;
  bool LeADAS_b_State2Complete = false;

  /* First, let's determine what we are going to do: */
  if (LeADAS_e_RobotState == E_Teleop)
  {
    /* Enable criteria goes here: */
    if (VsCONT_s_DriverInput.b_AutoBalance == true)
    {
      LeADAS_e_ActiveFeature = E_ADAS_DM_AutoBalance;
    }

    /* Abort criteria goes here: */
    if ((LeADAS_b_Driver1_JoystickActive == true) || (VeADAS_b_StateComplete == true))
    {
      /* Abort criteria goes here. */
      LeADAS_e_ActiveFeature = E_ADAS_Disabled;
      VeADAS_b_StateComplete = false;
    }
  }
  else if (LeADAS_e_RobotState == E_Auton)
  {
    if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind1)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_BlindLaunch;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_BlindLaunch) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DriveStraight;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DriveStraight) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootBlind2)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_ReverseAndIntake;
        V_ADAS_DriveTime = K_ADAS_DM_DriveTimeLong;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_ReverseAndIntake) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_Rotate180;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_Rotate180) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_BlindLaunch;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_BlindLaunch) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootAuto2)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_ReverseAndIntake;
        V_ADAS_DriveTime = K_ADAS_DM_DriveTimeShort;
        V_ADAS_DM_InitGyroAngle = L_Deg_GyroAngleDeg;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_ReverseAndIntake) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_BT_AutoBallTarget;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_BT_AutoBallTarget) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_RotateFieldOriented;
        V_ADAS_Deg_TargetAngle = L_Deg_GyroAngleDeg + 180;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDriveAndShootAuto3)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 1;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_1;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_1) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_BT_AutoBallTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_BT_2;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_BT_2) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 2;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_3;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_UT_4) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 3;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_PF_5;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_DM_PF_5) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_BT_AutoBallTarget;
        V_ADAS_Auton1State = E_ADAS_Auton_BT_6;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_BT_6) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_RotateFieldOriented;
        V_ADAS_Deg_TargetAngle = L_Deg_GyroAngleDeg + 180;
        V_ADAS_Auton1State = E_ADAS_Auton_DM_Rotate_7;
      }
      else if ((V_ADAS_Auton1State == E_ADAS_Auton_UT_8) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDeployCone) // Auton code for deplying Cones
    {
      // Step 1 - Place Cone
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_AutonDeployCone;
      }
      // Step 2
      else if ((LeADAS_e_ActiveFeature == E_ADAS_AutonDeployCone) && (VeADAS_b_StateComplete == true))
      {
        // LeADAS_e_ActiveFeature = AUTONCOMMAND;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDeployCube) // Auton code for deplying Cubes
    {
      // Step 1 - Place Cube
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_AutonDeployCube;
      }
      // Step 2
      else if ((LeADAS_e_ActiveFeature == E_ADAS_AutonDeployCube) && (VeADAS_b_StateComplete == true))
      {
        // LeADAS_e_ActiveFeature = AUTONCOMMAND;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDrivePath1) // Test Path
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower;
        V_ADAS_PathNum = 4;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonRotate)
    {
    }
    else
    {
      /* No auton requested. */
      LeADAS_e_ActiveFeature = E_ADAS_Disabled;
    }
  }
  else
  {
    LeADAS_e_ActiveFeature = E_ADAS_Disabled;
  }

  if (LeADAS_e_ActiveFeature == E_ADAS_Disabled)
  {
    /* Hmm, there was a transition, let's go ahead and reset all of the variables before we start: */
    ADAS_DM_Reset();
    VeADAS_b_StateComplete = false;
  }
toggle = frc::SmartDashboard::GetBoolean("movetotag", false);

if (toggle){
  LeADAS_e_ActiveFeature = E_ADAS_MoveToTag;
}
  // frc::SmartDashboard::PutNumber("current feature", (int)LeADAS_e_ActiveFeature);
  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_MoveToTag:
    // #ifdef unused
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_MoveToTag);
    *LeADAS_b_X_Mode = false;

    VeADAS_b_StateComplete = ADAS_DM_MoveToTag(L_Pct_FwdRev,
                                               L_Pct_Strafe,
                                               L_Pct_Rotate,
                                               L_OdomCentered,
                                               L_TagID,
                                               L_L_X_FieldPos,
                                               L_L_Y_FieldPos,
                                               L_VisionTargetingRequest,
                                               L_VisionTopTargetAquired,
                                               L_TagYawDegrees,
                                               LeLC_e_AllianceColor,
                                               L_CubeAlignCmd,
                                               L_ConeAlignCmd);

    break;
  case E_ADAS_DM_DriveStraight:
#ifdef unused
    VeADAS_b_StateComplete = ADAS_DM_DriveStraight(L_Pct_FwdRev,
                                                   L_Pct_Strafe,
                                                   L_Pct_Rotate,
                                                   L_RPM_Launcher,
                                                   L_Pct_Intake,
                                                   L_Pct_Elevator,
                                                   L_CameraUpperLightCmndOn,
                                                   L_CameraLowerLightCmndOn,
                                                   L_SD_RobotOriented);
#endif
    break;
  case E_ADAS_DM_Rotate180:
#ifdef unused
    VeADAS_b_StateComplete = ADAS_DM_Rotate180(L_Pct_FwdRev,
                                               L_Pct_Strafe,
                                               L_Pct_Rotate,
                                               L_RPM_Launcher,
                                               L_Pct_Intake,
                                               L_Pct_Elevator,
                                               L_CameraUpperLightCmndOn,
                                               L_CameraLowerLightCmndOn,
                                               L_SD_RobotOriented,
                                               L_Deg_GyroAngleDeg);
#endif
    break;
  case E_ADAS_DM_RotateFieldOriented:
#ifdef unused
    VeADAS_b_StateComplete = ADAS_DM_FieldOrientRotate(L_Pct_FwdRev,
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
#endif
    break;
  case E_ADAS_DM_PathFollower:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_PathFollower);

    VeADAS_b_StateComplete = ADAS_DM_PathFollower(L_Pct_FwdRev,
                                                  L_Pct_Strafe,
                                                  L_Pct_Rotate,
                                                  L_Pct_Intake,
                                                  L_SD_RobotOriented,
                                                  L_L_X_FieldPos,
                                                  L_L_Y_FieldPos,
                                                  L_Deg_GyroAngleDeg,
                                                  V_ADAS_PathNum,
                                                  V_ADAS_Auto_PathName);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
    break;
  case E_ADAS_DM_AutoBalance:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_AutoBalance);

    LeADAS_b_State2Complete = ADAS_DM_AutoBalance(L_Pct_FwdRev,
                                                  L_Pct_Strafe,
                                                  L_Pct_Rotate,
                                                  L_SD_RobotOriented,
                                                  LeADAS_b_X_Mode,
                                                  VeGRY_Deg_GyroRollAngleDegrees);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
    break;
  case E_ADAS_Disabled:
  default:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_PathFollower);
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *LeADAS_b_X_Mode = false;
    *L_VisionTargetingRequest = false;
    break;
  }

  return (LeADAS_e_ActiveFeature);
}