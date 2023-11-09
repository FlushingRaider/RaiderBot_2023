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
#include "ADAS_DM.hpp"
#include "ADAS_MN.hpp"
#include "Driver_inputs.hpp"
#include "Gyro.hpp"

#include "VisionV2.hpp"
#include "Odometry.hpp"
/* ADAS control state variables */
T_ADAS_ActiveFeature VeADAS_e_ActiveFeature = E_ADAS_Disabled;
T_ADAS_ActiveAutonFeature VeADAS_e_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
frc::SendableChooser<T_ADAS_ActiveAutonFeature> VeADAS_e_AutonChooser;
bool VeADAS_b_StateComplete = false;
bool VeADAS_b_State1Complete = false;
bool VeADAS_b_State2Complete = false;
bool VeADAS_b_AutonOncePerTrigger = false;


/* ADAS output control variables */
double VeADAS_Pct_SD_FwdRev = 0;
double VeADAS_Pct_SD_Strafe = 0;
double VeADAS_Pct_SD_Rotate = 0;
double VeADAS_Deg_SD_DesiredPose = 0;
bool VeADAS_b_SD_RobotOriented = false;
bool VeADAS_b_X_Mode = false;

bool VeADAS_b_CompletePrev = false;

double VeADAS_in_OffsetRequestX;
double VeADAS_in_OffsetRequestY;

double VeADAS_in_GlobalRequestX;
double VeADAS_in_GlobalRequestY;

bool VeADAS_b_CubeAlignButtonRequest;
bool VeADAS_b_CubeAlignButtonPrevious;

bool VeADAS_b_ConeAlignButtonRequest;
bool VeADAS_b_ConeAlignButtonPrevious;

bool toggle;

/******************************************************************************
 * Function:     ADAS_Main_Init
 *
 * Description:  Initialize all applicable ADAS variables at robot init.
 ******************************************************************************/
void ADAS_Main_Init(void)
{
  std::string_view LeADAS_Str_AutonSelectorName = "Auton";
  VeADAS_e_AutonChooser.AddOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  VeADAS_e_AutonChooser.AddOption("Drop Cube Drive FWD", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDropCubeDriveFwd);
  VeADAS_e_AutonChooser.AddOption("Deliver Cube Drive On Charge Station Auto Bal", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDeliverCubeDriveOnRampAutoBal);
  VeADAS_e_AutonChooser.AddOption("3 Cube Path Follower", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath1);
  VeADAS_e_AutonChooser.SetDefaultOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  frc::SmartDashboard::PutData(LeADAS_Str_AutonSelectorName, &VeADAS_e_AutonChooser);

  // VeADAS_e_AutonChooser.AddOption("Drive Straight", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveStraight);
  // VeADAS_e_AutonChooser.AddOption("Charge Station Auto Bal Goal", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDriveOverRampAutoBalV2);
  // VeADAS_e_AutonChooser.AddOption("Test Vision Auto", T_ADAS_ActiveAutonFeature::E_ADAS_TestVisionAuton);
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
  VeADAS_e_DriverRequestedAutonFeature = VeADAS_e_AutonChooser.GetSelected();
  frc::SmartDashboard::PutNumber("Requested Auton", float(VeADAS_e_DriverRequestedAutonFeature));
}

/******************************************************************************
 * Function:     ADAS_Main_Reset
 *
 * Description:  Reset all applicable ADAS variables.
 ******************************************************************************/
void ADAS_Main_Reset(void)
{
  VeADAS_e_ActiveFeature = E_ADAS_Disabled;
  VeADAS_Pct_SD_FwdRev = 0;
  VeADAS_Pct_SD_Strafe = 0;
  VeADAS_Pct_SD_Rotate = 0;

  VeADAS_b_SD_RobotOriented = false;
  VeADAS_e_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
  VeADAS_b_StateComplete = false;
  VeADAS_b_State1Complete = false;
  VeADAS_b_State2Complete = false;
  VeADAS_b_AutonOncePerTrigger = false;

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
T_ADAS_ActiveFeature ADAS_ControlMain(double                      *L_Pct_FwdRev,
                                      double                      *L_Pct_Strafe,
                                      double                      *L_Pct_Rotate,
                                      double                      *LeADAS_Deg_DesiredPose,
                                      bool                        *LeADAS_b_SD_RobotOriented,
                                      bool                        *LeADAS_b_X_Mode,
                                      bool                         LeADAS_b_Driver1_JoystickActive,
                                      bool                         L_Driver_SwerveGoalAutoCenter,
                                      double                       L_Deg_GyroAngleDeg,
                                      double                       L_L_X_FieldPos,
                                      double                       L_L_Y_FieldPos,
                                      T_RobotState                 LeADAS_e_RobotState,
                                      T_ADAS_ActiveFeature         LeADAS_e_ActiveFeature,
                                      bool                         L_OdomCentered,
                                      frc::DriverStation::Alliance LeLC_e_AllianceColor,
                                      double                       L_OdomOffsetX,
                                      double                       L_OdomOffsetY,
                                      double                       L_OdomGlobalRequestX,
                                      double                       L_OdomGlobalRequestY,
                                      double                       L_OdomOffsetRequestX,
                                      double                       L_OdomOffsetRequestY)
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
    else if (VeADAS_b_CubeAlignButtonRequest)
    {

      if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kBlue)
      {
        VeADAS_in_OffsetRequestX = C_TagScoreOffsetXCube;
      }
      else if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
      {
        VeADAS_in_OffsetRequestX = C_TagScoreOffsetXCube;
      }

      VeADAS_in_OffsetRequestY = C_TagScoreOffsetYCube;

      LeADAS_e_ActiveFeature = E_ADAS_MoveOffsetTag;
    }
    else if (VeADAS_b_ConeAlignButtonRequest)
    {

      if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kBlue)
      {
        VeADAS_in_OffsetRequestX = C_TagScoreOffsetXCone;
      }
      else if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
      {
        VeADAS_in_OffsetRequestX = C_TagScoreOffsetXCone;
      }

      VeADAS_in_OffsetRequestY = C_TagScoreOffsetYCone;

      // if (V_TagID == 1 || V_TagID == 8)
      // {
      //   VeADAS_in_OffsetRequestY = C_Tag1Y;
      // }
      // else if (V_TagID == 2 || V_TagID == 7)
      // {
      //   VeADAS_in_OffsetRequestY = C_Tag2Y;
      // }
      // else if (V_TagID == 3 || V_TagID == 6)
      // {
      //   VeADAS_in_OffsetRequestY = C_Tag3Y;
      // }

      LeADAS_e_ActiveFeature = E_ADAS_MoveOffsetTag;
    }

    /* Abort criteria goes here: */
    if ((LeADAS_b_Driver1_JoystickActive == true) || (VeADAS_b_StateComplete == true))
    {
      /* Abort criteria goes here. */

      VeADAS_b_CubeAlignButtonRequest = false;
      LeADAS_e_ActiveFeature = E_ADAS_Disabled;
      VeADAS_b_StateComplete = false;
      VeADAS_b_State1Complete = false;
      VeADAS_b_State2Complete = false;
    }
  }
  else if (LeADAS_e_RobotState == E_Auton)
  {
    if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDropCubeDriveFwd)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_MN_DeployHighCube;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MN_DeployHighCube) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollowerFWD;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollowerFWD) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDeliverCubeDriveOnRampAutoBal)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_MN_DeployHighCube; // Backup and put maniuplator into high cube drop position
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MN_DeployHighCube) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_MountDismountRamp; // With the bot semi mounted, auto balance
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_MountDismountRamp) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_AutoBalance; // Drive forward and over ramp
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_AutoBalance) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
    }
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_AutonDrivePath1) // Test Path
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_MN_DeployHighCube;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MN_DeployHighCube) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_MN_DeployMidCube;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MN_DeployMidCube) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
    }
    #ifdef unused
    else if (VeADAS_e_DriverRequestedAutonFeature == E_ADAS_TestVisionAuton)
    {
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false))
      {
        if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
        {
          VeADAS_in_GlobalRequestX = C_TagXred - C_TagScoreOffsetXCube;
        }
        else
        {
          VeADAS_in_GlobalRequestX = C_TagXblue +  C_TagScoreOffsetXCube;
        }
        VeADAS_in_GlobalRequestY = C_Tag1Y;
        LeADAS_e_ActiveFeature = E_ADAS_MoveGlobalTag;
      }
    }
    #endif
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

  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_MoveGlobalTag:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_MoveGlobalTag,
                                           E_ADAS_MAN_Driving);
    *LeADAS_b_X_Mode = false;

    VeADAS_b_StateComplete = MoveWithGlobalCoords(
        L_Pct_FwdRev,
        L_Pct_Strafe,
        L_Pct_Rotate,
        L_OdomCentered,
        V_TagYaw,
        L_L_X_FieldPos,
        L_L_Y_FieldPos,
        L_OdomGlobalRequestX,
        L_OdomGlobalRequestY);

    if (VeADAS_b_StateComplete)
    {
      // V_TagCentered = false; // we did what we needed with that tag snapshot, allow ourselves to take another later
    }
    break;
  case E_ADAS_MoveOffsetTag:
    // #ifdef unused
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_MoveOffsetTag,
                                           E_ADAS_MAN_Driving);
    *LeADAS_b_X_Mode = false;

    VeADAS_b_StateComplete = MoveWithOffsetTag(
        L_Pct_FwdRev,
        L_Pct_Strafe,
        L_Pct_Rotate,
        L_OdomCentered,
        V_TagYaw,
        L_OdomOffsetX,
        L_OdomOffsetY,
        L_OdomOffsetRequestX,
        L_OdomOffsetRequestY);

    if (VeADAS_b_StateComplete)
    {
      // V_TagCentered = false; // we did what we needed with that tag snapshot, allow ourselves to take another later
      VeADAS_b_CubeAlignButtonRequest = false;
      VeADAS_b_ConeAlignButtonRequest = false;
    }
    // frc::SmartDashboard::PutString("movetotagstep", V_MoveToTagStep);

    break;
  case E_ADAS_DM_MountDismountRamp:
      if (VeADAS_b_State1Complete == false)
    {
    VeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_MountDismountRamp,
                                           E_ADAS_MAN_Driving);
    }
    if (VeADAS_b_State2Complete == false)
    {
    VeADAS_b_State2Complete = ADAS_DM_DriveOntoStation(L_Pct_FwdRev,
                                                       L_Pct_Strafe,
                                                       L_Pct_Rotate,
                                                       LeADAS_b_SD_RobotOriented,
                                                       LeADAS_b_X_Mode,
                                                       VeGRY_Deg_GyroRollAngleDegrees);
    }

    VeADAS_b_StateComplete = (VeADAS_b_State1Complete == true && VeADAS_b_State2Complete == true);
    if (VeADAS_b_StateComplete == true) {VeADAS_b_State1Complete = false; VeADAS_b_State2Complete = false;}
    break;
  case E_ADAS_MN_DeployHighCube:
  case E_ADAS_MN_DeployMidCube:
  case E_ADAS_MN_DeployLowCube:
    if (VeADAS_b_State1Complete == false)
    {
    VeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           LeADAS_e_ActiveFeature,
                                           E_ADAS_MAN_Driving);
    }

    if (VeADAS_b_State2Complete == false)
    {
    VeADAS_b_State2Complete = ADAS_DM_Stop(L_Pct_FwdRev,
                                           L_Pct_Strafe,
                                           L_Pct_Rotate,
                                           LeADAS_b_SD_RobotOriented,
                                           KeADAS_t_DM_StopTm);
    }

    VeADAS_b_StateComplete = (VeADAS_b_State1Complete == true && VeADAS_b_State2Complete == true);
    if (VeADAS_b_StateComplete == true) {VeADAS_b_State1Complete = false; VeADAS_b_State2Complete = false;}
  break;
  case E_ADAS_DM_PathFollower1:
  case E_ADAS_DM_PathFollower2:
  case E_ADAS_DM_PathFollower3:
  case E_ADAS_DM_PathFollower4:
  case E_ADAS_DM_PathFollowerFWD:
    if (VeADAS_b_State1Complete == false)
    {
      VeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                             LeADAS_e_ActiveFeature,
                                             E_ADAS_MAN_Driving);
    }

    if (VeADAS_b_State2Complete == false)
    {
      VeADAS_b_State2Complete = ADAS_DM_PathFollower(L_Pct_FwdRev,
                                                     L_Pct_Strafe,
                                                     L_Pct_Rotate,
                                                     LeADAS_Deg_DesiredPose,
                                                     LeADAS_b_SD_RobotOriented,
                                                     L_L_X_FieldPos,
                                                     L_L_Y_FieldPos,
                                                     L_Deg_GyroAngleDeg,
                                                     LeADAS_e_ActiveFeature,
                                                     LeLC_e_AllianceColor);
    }

    VeADAS_b_StateComplete = (VeADAS_b_State1Complete == true && VeADAS_b_State2Complete == true);

    if (VeADAS_b_StateComplete == true) {VeADAS_b_State1Complete = false; VeADAS_b_State2Complete = false;}
  break;
  case E_ADAS_DM_AutoBalance:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_DM_AutoBalance,
                                           E_ADAS_MAN_Driving);

    LeADAS_b_State2Complete = ADAS_DM_AutoBalance(L_Pct_FwdRev,
                                                  L_Pct_Strafe,
                                                  L_Pct_Rotate,
                                                  LeADAS_b_SD_RobotOriented,
                                                  LeADAS_b_X_Mode,
                                                  VeGRY_Deg_GyroRollAngleDegrees);

    VeADAS_b_StateComplete = (LeADAS_b_State1Complete == true && LeADAS_b_State2Complete == true);
    break;
  case E_ADAS_Disabled:
  default:
    LeADAS_b_State1Complete = ADAS_MN_Main(LeADAS_e_RobotState,
                                           E_ADAS_Disabled,
                                           E_ADAS_MAN_Driving);
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *LeADAS_b_X_Mode = false;
    break;
  }

  return (LeADAS_e_ActiveFeature);
}