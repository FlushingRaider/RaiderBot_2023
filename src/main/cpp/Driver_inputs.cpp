/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls. 
  This is where the "CONT" variables are set.  Everything gets mapped to the structure VsCONT_s_DriverInput.
 */
#include "Enums.hpp"
#include "Lookup.hpp"
#include <math.h>

RobotUserInput VsCONT_s_DriverInput;

/******************************************************************************
 * Function:     Joystick1_robot_mapping
 *
 * Description:  Captures and maps driver inputs from controller 1.
 ******************************************************************************/
void Joystick1_robot_mapping(bool    LeCONT_b_Driver1ButtonBack,
                             bool    LeCONT_b_Driver1ButtonStart,
                             double  LeCONT_Cmd_Driver1LeftAxisY,
                             double  LeCONT_Cmd_Driver1LeftAxisX,
                             double  LeCONT_Cmd_Driver1RightAxisX,
                             double  LeCONT_Cmd_Driver1LeftTriggerAxis,
                             bool    LeCONT_b_Driver1ButtonA,
                             bool    LeCONT_b_Driver1ButtonX,
                             bool    LeCONT_b_Driver1ButtonY,
                             bool    LeCONT_b_Driver1ButtonRB,
                             bool    LeCONT_b_Driver1ButtonB,
                             bool    LeCONT_b_Driver1ButtonLB,
                             int     LeCONT_Deg_Driver1POV)
  {
  double                LeCONT_Pct_AxisTotal         = 0;
  bool                  LeCONT_b_JoystickActive      = false;
  T_TurretCmndDirection LeCONT_e_TurretCmndDirection = E_TurrentCmndNone;

  VsCONT_s_DriverInput.b_ZeroGyro                      = (LeCONT_b_Driver1ButtonBack || LeCONT_b_Driver1ButtonStart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  VsCONT_s_DriverInput.pct_SwerveForwardBack           = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftAxisY);  // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.pct_SwerveStrafe                = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftAxisX);        // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.deg_SwerveRotate                = ScaleJoystickAxis(LeCONT_Cmd_Driver1RightAxisX);      // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.v_SwerveSpeed                   = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftTriggerAxis);  // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.b_SwerveGoalAutoCenter          = LeCONT_b_Driver1ButtonA;
  VsCONT_s_DriverInput.b_SwerveRotateTo0               = LeCONT_b_Driver1ButtonX;
  VsCONT_s_DriverInput.b_SwerveRotateTo90              = LeCONT_b_Driver1ButtonY;
  VsCONT_s_DriverInput.b_CameraLight                   = LeCONT_b_Driver1ButtonRB;                      //Controller 1, X button (3), when held, turns on the camera light
  VsCONT_s_DriverInput.b_AutoIntake                    = LeCONT_b_Driver1ButtonB;
  VsCONT_s_DriverInput.b_VisionDriverModeOverride      = LeCONT_b_Driver1ButtonLB;
   
  LeCONT_Pct_AxisTotal = (fabs(VsCONT_s_DriverInput.pct_SwerveStrafe) + fabs(VsCONT_s_DriverInput.deg_SwerveRotate) + fabs(VsCONT_s_DriverInput.v_SwerveSpeed));
  
  if (LeCONT_Pct_AxisTotal > 0)
    {
    LeCONT_b_JoystickActive = true;
    }

  VsCONT_s_DriverInput.b_JoystickActive = LeCONT_b_JoystickActive;

  if (LeCONT_Deg_Driver1POV == 270)
    {
    LeCONT_e_TurretCmndDirection = E_TurrentCmndLeft;
    }
  else if (LeCONT_Deg_Driver1POV == 90)
    {
    LeCONT_e_TurretCmndDirection = E_TurrentCmndRight;
    }
  else
    {
    LeCONT_e_TurretCmndDirection = E_TurrentCmndNone;
    }

   VsCONT_s_DriverInput.e_TurretCmndDirection = LeCONT_e_TurretCmndDirection;
  }

/******************************************************************************
 * Function:     Joystick2_robot_mapping
 *
 * Description:  Captures and maps driver inputs from controller 2.
 ******************************************************************************/
void Joystick2_robot_mapping(bool    LeCONT_b_Driver2ButtonA,
                             bool    LeCONT_b_Driver2ButtonB,
                             bool    LeCONT_b_Driver2ButtonRB,
                             bool    LeCONT_b_Driver2ButtonLB,
                             bool    LeCONT_b_Driver2ButtonStart,
                             bool    LeCONT_b_Driver2ButtonX,
                             bool    LeCONT_b_Driver2ButtonY,
                             double  LeCONT_Cmd_Driver2LeftAxisY,
                             double  LeCONT_Cmd_Driver2RightAxisY,
                             int     LeCONT_Deg_Driver2POV,
                             bool    LeCONT_b_Driver2ButtonBack)
  {
  TeLFT_e_LiftCmndDirection   LeCONT_e_LiftCmndDirection     = E_LiftCmndNone;

  VsCONT_s_DriverInput.b_ElevatorUp                    = LeCONT_b_Driver2ButtonA;      //Controller 2, A button (1), (robot.cpp) Elevator goes up
  VsCONT_s_DriverInput.b_ElevatorDown                  = LeCONT_b_Driver2ButtonB;      //Controller 2, B button (2), (robot.cpp) Elevator goes down
  VsCONT_s_DriverInput.b_StopShooterAutoClimbResetGyro = LeCONT_b_Driver2ButtonLB;     //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain, pauses auto climb and resets encoders in test mode
  VsCONT_s_DriverInput.b_AutoSetSpeedShooter           = LeCONT_b_Driver2ButtonStart;  //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  VsCONT_s_DriverInput.pct_ManualShooterDesiredSpeed   = LeCONT_Cmd_Driver2LeftAxisY;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  VsCONT_s_DriverInput.b_LiftControl                   = LeCONT_b_Driver2ButtonRB;     //Controller 2, X button (3), (Manipulator.cpp) starts automated states machine
  VsCONT_s_DriverInput.b_IntakeIn                      = LeCONT_b_Driver2ButtonX;      //Controller 2 (3), controlls the intake in on trigger pressed
  VsCONT_s_DriverInput.b_IntakeOut                     = LeCONT_b_Driver2ButtonY;      //Controller 2 (4), controlls the intake out on trigger pressed

  if (LeCONT_Deg_Driver2POV == 0)
    {
    LeCONT_e_LiftCmndDirection = E_LiftCmndUp;
    }
  else if (LeCONT_Deg_Driver2POV == 180)
    {
    LeCONT_e_LiftCmndDirection = E_LiftCmndDown;
    }
  else if (LeCONT_Deg_Driver2POV == 90)
    {
    LeCONT_e_LiftCmndDirection = E_LiftCmndForward;
    }
  else if (LeCONT_Deg_Driver2POV == 270)
    {
    LeCONT_e_LiftCmndDirection = E_LiftCmndBack;
    }
  else
    {
    LeCONT_e_LiftCmndDirection = E_LiftCmndNone;
    }

  VsCONT_s_DriverInput.e_LiftCmndDirection = LeCONT_e_LiftCmndDirection;
  }