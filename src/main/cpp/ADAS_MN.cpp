/*
  ADAS_MN.cpp

  Created on: Feb 21, 2023
  Author: Jay L

  ADAS (Advanced Driver-Assistance Systems) Manipulator
  Contains the logic and code used for the Manipulator control:
    - Schedules states in a non-linear state machine
  Changes:
  2023-02-21 -> Alpha
 */


#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "control_pid.hpp"
#include "rev/CANSparkMax.h"
#include "Const.hpp"
#include "Lookup.hpp"
#include "DriveControl.hpp"
#include "Manipulator.hpp"
#include "Driver_inputs.hpp"


TeMAN_ManipulatorStates VeADAS_e_MAN_SchedState = E_MAN_Init; //State Scheduled in relation to driver input. Used for non-linear state machines
bool                    VeADAS_b_MAN_DropObjectSlow = false;
bool                    VeADAS_b_MAN_DropObjectFast = false;

double                  VeADAS_t_MAN_DropObjectTm = 0.0;  // Timer that will keep rollers on for a specific amount of time

/* Configuration cals: */


/******************************************************************************
 * Function:     ADAS_MN_ConfigsInit
 *
 * Description:  Contains the configurations for the UT.
 ******************************************************************************/
void ADAS_MN_ConfigsInit()
{
  // set coefficients
 
#ifdef ADAS_MN_Test
  // display coefficients on SmartDashboard

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
  VeADAS_t_MAN_DropObjectTm = 0.0;
}

/******************************************************************************
 * Function:    ManipulatorScheduelerTeleop
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator
 ******************************************************************************/
 bool ManipulatorScheduelerTeleop (void)
  {
  bool                    LeADAS_b_MAN_DropObjectSlow = false;
  bool                    LeADAS_b_MAN_DropObjectFast = false;
  bool                    LeADAS_b_MAN_StateComplete = false;

  if (VsCONT_s_DriverInput.b_DrivingPosition == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_Driving;
    }
  else if (VsCONT_s_DriverInput.b_FrontHighCube == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_HighCubeDrop;
    }
  else if (VsCONT_s_DriverInput.b_FrontLowCube == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_LowCubeDrop;
    }
      else if (VsCONT_s_DriverInput.b_FrontLowCone == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_LowConeDrop;
    }
      else if (VsCONT_s_DriverInput.b_FrontHighCone == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_HighConeDrop;
    }
      else if (VsCONT_s_DriverInput.b_MidIntakeOut == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_MidConeIntake;
    }
      else if (VsCONT_s_DriverInput.b_FloorConeDrop == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_FloorConeDrop;
    }
      else if (VsCONT_s_DriverInput.b_InitState == true)
    {
      VeADAS_e_MAN_SchedState = E_MAN_Init;
    }
  else
    {
      /* No updates */
    }

  if (VsCONT_s_DriverInput.b_DropGamePieceFast == true)
    {
      LeADAS_b_MAN_DropObjectFast = true;
      VeADAS_t_MAN_DropObjectTm = C_ExeTime;
    }
  else if (VsCONT_s_DriverInput.b_DropGamePieceSlow == true)
    {
      LeADAS_b_MAN_DropObjectSlow = true;
      VeADAS_t_MAN_DropObjectTm = C_ExeTime;
    }
  else if ((VeADAS_t_MAN_DropObjectTm > 0) &&
           (VeADAS_t_MAN_DropObjectTm <= KeMAN_t_GripperOnTm))
    {
      LeADAS_b_MAN_DropObjectSlow = VeADAS_b_MAN_DropObjectSlow;
      LeADAS_b_MAN_DropObjectFast = VeADAS_b_MAN_DropObjectFast;
      VeADAS_t_MAN_DropObjectTm += C_ExeTime;
    }
  else
    {
      LeADAS_b_MAN_DropObjectSlow = false;
      LeADAS_b_MAN_DropObjectFast = false;
      VeADAS_t_MAN_DropObjectTm = 0.0;
    }

  if (VeADAS_e_MAN_SchedState == VeMAN_e_AttndState)
    {
      LeADAS_b_MAN_StateComplete = true;
    }

  VeADAS_b_MAN_DropObjectSlow = LeADAS_b_MAN_DropObjectSlow;
  VeADAS_b_MAN_DropObjectFast = LeADAS_b_MAN_DropObjectFast;

  return(LeADAS_b_MAN_StateComplete);
  }


/******************************************************************************
 * Function:    ManipulatorScheduelerAutonBasic
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator in auton.
 ******************************************************************************/
 bool ManipulatorScheduelerAutonBasic(void)
  {
    bool LeADAS_b_MAN_StateComplete = false;
    TeMAN_ManipulatorStates LeADAS_e_MAN_State = E_MAN_Driving;

    VeADAS_e_MAN_SchedState = LeADAS_e_MAN_State;
    VeADAS_b_MAN_DropObjectSlow = false;
    VeADAS_b_MAN_DropObjectFast = false;

    if (LeADAS_e_MAN_State == VeMAN_e_AttndState)
      {
        LeADAS_b_MAN_StateComplete = true;
      }
    return(LeADAS_b_MAN_StateComplete);
  }

/******************************************************************************
 * Function:    ManipulatorScheduelerAutonAction
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator in auton.
 ******************************************************************************/
 bool ManipulatorScheduelerAutonAction(TeADAS_AutonManipulatorStates LeADAS_e_MAN_StateReq)
  {
    bool                    LeADAS_b_MAN_StateComplete = false;
    TeMAN_ManipulatorStates LeADAS_e_MAN_State         = E_MAN_Driving;
    bool                    LeADAS_b_MAN_DropSlow      = false;
    bool                    LeADAS_b_MAN_DropFast      = false;
    bool                    LeADAS_b_MAN_DropCmplt     = false;

    if (LeADAS_e_MAN_StateReq == E_ADAS_MAN_Driving)
      {
        LeADAS_e_MAN_State = E_MAN_Driving;
        LeADAS_b_MAN_DropCmplt = true;
      }
    else if (LeADAS_e_MAN_StateReq == E_ADAS_MAN_HighCubeDropPosition)
      {
        LeADAS_e_MAN_State = E_MAN_HighCubeDrop;
        LeADAS_b_MAN_DropFast = true;
        
        if (VeMAN_e_AttndState == LeADAS_e_MAN_State)
          {
            VeADAS_t_MAN_DropObjectTm += C_ExeTime;
          }

        if (VeADAS_t_MAN_DropObjectTm >= KeMAN_t_GripperOnTm)
          {
            LeADAS_b_MAN_DropCmplt = true;
            LeADAS_b_MAN_DropFast = false;
          }
      }
    else if (LeADAS_e_MAN_StateReq == E_ADAS_MAN_MidCubeDropPosition)
      {
        LeADAS_e_MAN_State = E_MAN_LowCubeDrop;
        LeADAS_b_MAN_DropFast = true;
        
        if (VeMAN_e_AttndState == LeADAS_e_MAN_State)
          {
            VeADAS_t_MAN_DropObjectTm += C_ExeTime;
          }

        if (VeADAS_t_MAN_DropObjectTm >= KeMAN_t_GripperOnTm)
          {
            LeADAS_b_MAN_DropCmplt = true;
            LeADAS_b_MAN_DropFast = false;
          }
      }
    else if (LeADAS_e_MAN_StateReq == E_ADAS_MAN_LowCubeDropPosition)
      {
        LeADAS_e_MAN_State = E_MAN_Driving;
        LeADAS_b_MAN_DropFast = true;
        
        if (VeMAN_e_AttndState == LeADAS_e_MAN_State)
          {
            VeADAS_t_MAN_DropObjectTm += C_ExeTime;
          }

        if (VeADAS_t_MAN_DropObjectTm >= KeMAN_t_GripperOnTm)
          {
            LeADAS_b_MAN_DropCmplt = true;
            LeADAS_b_MAN_DropFast = false;
          }
      }
    else
      {
        LeADAS_b_MAN_DropCmplt = true;
      }

    VeADAS_e_MAN_SchedState = LeADAS_e_MAN_State;
    VeADAS_b_MAN_DropObjectSlow = LeADAS_b_MAN_DropSlow;
    VeADAS_b_MAN_DropObjectFast = LeADAS_b_MAN_DropFast;

    if ((LeADAS_e_MAN_State == VeMAN_e_AttndState) &&
        (LeADAS_b_MAN_DropCmplt == true))
      {
        LeADAS_b_MAN_StateComplete = true;
        VeADAS_t_MAN_DropObjectTm = 0;
      }
    return(LeADAS_b_MAN_StateComplete);
  }


/******************************************************************************
 * Function:     ADAS_MN_Main
 *
 * Description:  Manages and controls the manipulator controls.
 ******************************************************************************/
bool ADAS_MN_Main(T_RobotState                  L_RobotState,
                  T_ADAS_ActiveFeature          LeADAS_e_ActiveFeature,
                  TeADAS_AutonManipulatorStates LeADAS_e_MAN_ReqAction)
{
  bool LeADAS_b_MN_Complete = false;

  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_DM_PathFollower1:
  break;
  case E_ADAS_DM_PathFollower2:
  case E_ADAS_DM_PathFollower3:
  case E_ADAS_DM_PathFollower4:
      LeADAS_e_MAN_ReqAction = E_ADAS_MAN_Driving;
  break;
  case E_ADAS_MN_DeployHighCube:
      LeADAS_e_MAN_ReqAction = E_ADAS_MAN_HighCubeDropPosition;
  break;
  case E_ADAS_MN_DeployMidCube:
      LeADAS_e_MAN_ReqAction = E_ADAS_MAN_MidCubeDropPosition;
  break;
  case E_ADAS_MN_DeployLowCube:
      LeADAS_e_MAN_ReqAction = E_ADAS_MAN_LowCubeDropPosition;
  break;
  default:
  break;
  }

  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_Disabled:
    LeADAS_b_MN_Complete = ManipulatorScheduelerTeleop();
  break;
  
  case E_ADAS_DM_StopDeployCube:
  case E_ADAS_MN_DeployHighCube:
  case E_ADAS_MN_DeployMidCube:
  case E_ADAS_MN_DeployLowCube:
  case E_ADAS_DM_PathFollower1:
  case E_ADAS_DM_PathFollower2:
  case E_ADAS_DM_PathFollower3:
  case E_ADAS_DM_PathFollower4:
    LeADAS_b_MN_Complete = ManipulatorScheduelerAutonAction(LeADAS_e_MAN_ReqAction);
  break;

  case E_ADAS_DM_AutoBalance:
  case E_ADAS_MoveOffsetTag:
  default:
    LeADAS_b_MN_Complete = ManipulatorScheduelerAutonBasic();
  break;
  }

  return (LeADAS_b_MN_Complete);
}