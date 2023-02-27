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
#include "rev/CANSparkMax.h"
#include "Lookup.hpp"
#include "Const.hpp"
#include <frc/DriverStation.h>
#include "DriveControl.hpp"
#include "Manipulator.hpp"
#include "Driver_inputs.hpp"


TeMAN_ManipulatorStates VeADAS_e_MAN_SchedState = E_MAN_Init; //State Scheduled in relation to driver input. Used for non-linear state machines
bool                    VeADAS_b_MAN_DropObject = false;

double                  VeADAS_t_MAN_DropObjectTm = 0.0;  // Timer that will keep rollers on for a specific amount of time

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
  TeMAN_ManipulatorStates VeADAS_e_AttndState = E_MAN_Init;
  V_ADAS_MN_DebounceTime = 0;
  V_ADAS_MN_RotateErrorPrev = 0;
  V_ADAS_MN_LauncherSpeedPrev = 0;
  V_ADAS_MN_TargetAquiredPrev = false;
}

/******************************************************************************
 * Function:    ManipulatorScheduelerTeleop
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator
 ******************************************************************************/
 bool ManipulatorScheduelerTeleop (void)
  {
  TeMAN_ManipulatorStates LeADAS_e_MAN_State      = VeADAS_e_MAN_SchedState;
  bool                    LeADAS_b_MAN_DropObject = false;
  bool                    LeADAS_b_MAN_StateComplete = false;

  if (VsCONT_s_DriverInput.b_MainIntakeOut == true)
    {
      LeADAS_e_MAN_State = E_MAN_MainIntake;
    }
  else if (VsCONT_s_DriverInput.b_DrivingPosition == true)
    {
      LeADAS_e_MAN_State = E_MAN_Driving;
    }
  else if (VsCONT_s_DriverInput.b_HighPositionCube == true)
    {
      LeADAS_e_MAN_State = E_MAN_PositioningHighCube;
    }
  else if (VsCONT_s_DriverInput.b_LowPositionCube == true)
    {
      LeADAS_e_MAN_State = E_MAN_PositioningLowCube;
    }
  else if (VsCONT_s_DriverInput.b_HighPositionCone == true)
    {
      LeADAS_e_MAN_State = E_MAN_PositioningHighCone;
    }
  else if (VsCONT_s_DriverInput.b_LowPositionCone == true)
    {
      LeADAS_e_MAN_State = E_MAN_PositioningLowCone;
    }
  else if (VsCONT_s_DriverInput.b_IntakeArmIn = true)
    {
      LeADAS_e_MAN_State = E_MAN_Driving;
    }
  else if (VsCONT_s_DriverInput.b_ArmDown = true)
    {
      LeADAS_e_MAN_State = E_MAN_FloorIntake;
    }
  else
    {
      /* No updates */
    }

  if ((VsCONT_s_DriverInput.b_DropGamePiece == true) ||
       ((VeADAS_t_MAN_DropObjectTm > 0) &&
        (VeADAS_t_MAN_DropObjectTm <= KeMAN_t_GripperOnTm)))
    {
      LeADAS_b_MAN_DropObject = true;
      VeADAS_t_MAN_DropObjectTm += C_ExeTime;
    }
  else
    {
      LeADAS_b_MAN_DropObject = false;
      VeADAS_t_MAN_DropObjectTm = 0.0;
    }

  if (LeADAS_e_MAN_State == VeMAN_e_AttndState)
    {
      LeADAS_b_MAN_StateComplete = true;
    }

    VeADAS_e_MAN_SchedState = LeADAS_e_MAN_State;
    VeADAS_b_MAN_DropObject = LeADAS_b_MAN_DropObject;

    return(LeADAS_b_MAN_StateComplete);
  }


/******************************************************************************
 * Function:    ManipulatorScheduelerAuton
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator in auton.
 ******************************************************************************/
 bool ManipulatorScheduelerAuton(void)
  {
    bool LeADAS_b_MAN_StateComplete = false;
    TeMAN_ManipulatorStates LeADAS_e_MAN_State = E_MAN_Driving;

    VeADAS_e_MAN_SchedState = LeADAS_e_MAN_State;
    VeADAS_b_MAN_DropObject = false;

    if (LeADAS_e_MAN_State == VeMAN_e_AttndState)
      {
        LeADAS_b_MAN_StateComplete = true;
      }
    return(LeADAS_b_MAN_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_MN_Main
 *
 * Description:  Manages and controls the manipulator controls.
 ******************************************************************************/
bool ADAS_MN_Main(T_RobotState         L_RobotState,
                  T_ADAS_ActiveFeature LeADAS_e_ActiveFeature)
{
  bool LeADAS_b_MN_Complete = false;

  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_Disabled:
    LeADAS_b_MN_Complete = ManipulatorScheduelerTeleop();
  break;

  case E_ADAS_DM_CubeAlign:
  case E_ADAS_DM_ConeAlign:
  case E_ADAS_DM_AutoBalance:
  case E_ADAS_MoveToTag:
  default:
    LeADAS_b_MN_Complete = ManipulatorScheduelerAuton();
  break;
  }

  return (LeADAS_b_MN_Complete);
}