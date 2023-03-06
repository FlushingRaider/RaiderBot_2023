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

#include "VisionV2.hpp"

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

double VeADAS_in_OffsetRequestX;
double VeADAS_in_OffsetRequestY;

double VeADAS_in_GlobalRequestX;
double VeADAS_in_GlobalRequestY;

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
  V_ADAS_AutonChooser.AddOption("Drive Straight", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveStraight);
  V_ADAS_AutonChooser.AddOption("Drop Cube Drive FWD", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDropCubeDriveFwd);
  V_ADAS_AutonChooser.AddOption("Charge Station Auto Bal Goal", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveOverRampAutoBalV2);
  V_ADAS_AutonChooser.AddOption("Test Path", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath1);
  V_ADAS_AutonChooser.SetDefaultOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  frc::SmartDashboard::PutData(L_AutonSelectorName, &V_ADAS_AutonChooser);

  // frc::SmartDashboard::PutBoolean("movetotag", toggle);
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
                                      double L_OdomOffsetX,
                                      double L_OdomOffsetY,
                                      double L_OdomGlobalRequestX,
                                      double L_OdomGlobalRequestY,
                                      double L_OdomOffsetRequestX,
                                      double L_OdomOffsetRequestY)
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
    if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDriveStraight)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
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
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDropCubeDriveFwd)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
       LeADAS_e_ActiveFeature = E_ADAS_DM_DriveRevStraight;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DriveRevStraight) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DriveStraightFar;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DriveStraightFar) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        V_ADAS_AutonOncePerTrigger = true;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDriveOverRampAutoBalV2)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) &&
          (VeADAS_b_StateComplete == false) &&
          (V_ADAS_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DriveRevStraight;   // Backup to push cube into goal
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DriveRevStraight) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_MountDismountRamp;  // Drive forward and over ramp
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_MountDismountRamp) &&
               (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_AutoBalance;  // With the bot semi mounted, auto balance
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_AutoBalance) &&
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
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_MoveOffsetTag)
    {
      LeADAS_e_ActiveFeature = E_ADAS_MoveOffsetTag;
      VeADAS_in_OffsetRequestX = 168.0; // 14ft
      if (V_TagID == 1 || V_TagID == 8)
      {
        VeADAS_in_OffsetRequestY = C_Tag1Y;
      }
      if (V_TagID == 3 || V_TagID == 6)
      {
        VeADAS_in_OffsetRequestY = C_Tag3Y;
      }
    }
    else if (VsCONT_s_DriverInput.b_CubeAlign && LeADAS_e_ActiveFeature == E_ADAS_Disabled)
    {

      if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kBlue)
      {
        VeADAS_in_OffsetRequestX = C_TagXblue - C_TagScoreOffset;
      }
      else if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
      {
        VeADAS_in_OffsetRequestX = C_TagXred + C_TagScoreOffset;
      }

      if (V_TagID == 1 || V_TagID == 8)
      {
        VeADAS_in_OffsetRequestY = C_Tag1Y;
      }
      else if(V_TagID == 2 || V_TagID == 7){
        VeADAS_in_OffsetRequestY = C_Tag2Y;
      }
      else if (V_TagID == 3 || V_TagID == 6)
      {
        VeADAS_in_OffsetRequestY = C_Tag3Y;
      }      

      LeADAS_e_ActiveFeature = E_ADAS_MoveOffsetTag;
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

  // toggle = frc::SmartDashboard::GetBoolean("movetotag", false);
  // testXOffset = frc::SmartDashboard::GetNumber("test x", 46.0);
  // testYOffset = frc::SmartDashboard::GetNumber("test y", 3.0);

  // if (toggle)
  // {
  //   LeADAS_e_ActiveFeature = E_ADAS_MoveOffsetTag;
  // }
  // frc::SmartDashboard::PutNumber("current feature", (int)LeADAS_e_ActiveFeature);

  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_MoveGlobalTag:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_MoveGlobalTag);
    *LeADAS_b_X_Mode = false;

    VeADAS_b_StateComplete = MoveWithGlobalCoords(
        L_Pct_FwdRev,
        L_Pct_Strafe,
        L_Pct_Rotate,
        L_OdomCentered,
        L_TagYawDegrees,
        L_L_X_FieldPos,
        L_L_Y_FieldPos,
        L_OdomGlobalRequestX,
        L_OdomGlobalRequestY);

    if (VeADAS_b_StateComplete)
    {
      V_TagCentered = false; // we did what we needed with that tag snapshot, allow ourselves to take another later
    }
    break;
  case E_ADAS_MoveOffsetTag:
    // #ifdef unused
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_MoveOffsetTag);
    *LeADAS_b_X_Mode = false;

    VeADAS_b_StateComplete = MoveWithOffsetTag(
        L_Pct_FwdRev,
        L_Pct_Strafe,
        L_Pct_Rotate,
        L_OdomCentered,
        L_TagYawDegrees,
        L_OdomOffsetX,
        L_OdomOffsetY,
        L_OdomOffsetRequestX,
        L_OdomOffsetRequestY);

    if (VeADAS_b_StateComplete)
    {
      V_TagCentered = false; // we did what we needed with that tag snapshot, allow ourselves to take another later
    }
    // frc::SmartDashboard::PutString("movetotagstep", V_MoveToTagStep);

    break;
  case E_ADAS_DM_DriveStraight:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_DriveStraight);

    LeADAS_b_State2Complete = ADAS_DM_DriveStraight(L_Pct_FwdRev,
                                                   L_Pct_Strafe,
                                                   L_Pct_Rotate,
                                                   L_SD_RobotOriented);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
  break;
    case E_ADAS_DM_DriveStraightFar:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_DriveStraightFar);

    LeADAS_b_State2Complete = ADAS_DM_DriveStraightFar(L_Pct_FwdRev,
                                                      L_Pct_Strafe,
                                                      L_Pct_Rotate,
                                                      L_SD_RobotOriented);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
  break;
  case E_ADAS_DM_DriveRevStraight:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_DriveRevStraight);

    LeADAS_b_State2Complete = ADAS_DM_DriveRevStraight(L_Pct_FwdRev,
                                                       L_Pct_Strafe,
                                                       L_Pct_Rotate,
                                                       L_SD_RobotOriented);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
  break;
  case E_ADAS_DM_MountDismountRamp:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_MountDismountRamp);

    LeADAS_b_State2Complete = ADAS_DM_DriveOntoStation(L_Pct_FwdRev,
                                                       L_Pct_Strafe,
                                                       L_Pct_Rotate,
                                                       L_SD_RobotOriented,
                                                       LeADAS_b_X_Mode,
                                                       VeGRY_Deg_GyroRollAngleDegrees);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
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
                                           E_ADAS_Disabled);
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *LeADAS_b_X_Mode = false;
    *L_VisionTargetingRequest = false;
    break;
  }

  return (LeADAS_e_ActiveFeature);
}