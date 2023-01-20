/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls. 
 */
#include "Enums.hpp"
#include "Lookup.hpp"
#include <frc/Joystick.h>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotUserInput VsDriverInput;

/******************************************************************************
 * Function:     Joystick_robot_mapping
 *
 * Description:  Captures and maps driver inputs.
 ******************************************************************************/
void Joystick_robot_mapping(bool    LeCont_b_Driver2ButtonA,
                            bool    LeCont_b_Driver2ButtonB,
                            bool    LeCont_b_Driver2ButtonRB,
                            bool    LeCont_b_Driver2ButtonLB,
                            bool    LeCont_b_Driver2ButtonStart,
                            bool    LeCont_b_Driver1ButtonBack,
                            bool    LeCont_b_Driver1ButtonStart,
                            bool    LeCont_b_Driver2ButtonX,
                            bool    LeCont_b_Driver2ButtonY,
                            double  LeCont_Cmd_Driver2LeftAxisY,
                            double  LeCont_Cmd_Driver2RightAxisY,
                            double  LeCont_Cmd_Driver1LeftAxisY,
                            double  LeCont_Cmd_Driver1LeftAxisX,
                            double  LeCont_Cmd_Driver1RightAxisX,
                            double  LeCont_Cmd_Driver1LeftTriggerAxis,
                            bool    LeCont_b_Driver1ButtonA,
                            bool    LeCont_b_Driver1ButtonX,
                            bool    LeCont_b_Driver1ButtonY,
                            int     LeCont_Deg_Driver2POV,
                            bool    LeCont_b_Driver1ButtonRB,
                            bool    LeCont_b_Driver1ButtonB,
                            bool    LeCont_b_Driver1ButtonLB,
                            bool    LeCont_b_Driver2ButtonBack,
                            int     LeCont_Deg_Driver1POV)
  {
  double                L_AxisTotal             = 0;
  bool                  L_JoystickActive        = false;
  TeLFT_e_LiftCmndDirection   L_LiftCmndDirection     = E_LiftCmndNone;
  T_TurretCmndDirection L_e_TurretCmndDirection = E_TurrentCmndNone;


  VsDriverInput.b_ElevatorUp                    = LeCont_b_Driver2ButtonA;                       //Controller 2, A button (1), (robot.cpp) Elevator goes up
  VsDriverInput.b_ElevatorDown                  = LeCont_b_Driver2ButtonB;                     //Controller 2, B button (2), (robot.cpp) Elevator goes down
  VsDriverInput.b_ZeroGyro                      = (LeCont_b_Driver1ButtonBack || LeCont_b_Driver1ButtonStart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  VsDriverInput.b_StopShooterAutoClimbResetGyro = LeCont_b_Driver2ButtonLB;     //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain, pauses auto climb and resets encoders in test mode
  VsDriverInput.b_AutoSetSpeedShooter           = LeCont_b_Driver2ButtonStart;         //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  VsDriverInput.pct_ManualShooterDesiredSpeed   = LeCont_Cmd_Driver2LeftAxisY;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  VsDriverInput.b_LiftControl                   = LeCont_b_Driver2ButtonRB;                     //Controller 2, X button (3), (Lift.cpp) starts automated states machine
  VsDriverInput.b_IntakeIn                      = LeCont_b_Driver2ButtonX;                         //Controller 2 (3), controlls the intake in on trigger pressed
  VsDriverInput.b_IntakeOut                     = LeCont_b_Driver2ButtonY;                         //Controller 2 (4), controlls the intake out on trigger pressed
  VsDriverInput.pct_SwerveForwardBack           = ScaleJoystickAxis(LeCont_Cmd_Driver1LeftAxisY);  // Scale the axis, also used for debouncing
  VsDriverInput.pct_SwerveStrafe                = ScaleJoystickAxis(LeCont_Cmd_Driver1LeftAxisX);        // Scale the axis, also used for debouncing
  VsDriverInput.deg_SwerveRotate                = ScaleJoystickAxis(LeCont_Cmd_Driver1RightAxisX);      // Scale the axis, also used for debouncing
  VsDriverInput.v_SwerveSpeed                   = ScaleJoystickAxis(LeCont_Cmd_Driver1LeftTriggerAxis);  // Scale the axis, also used for debouncing
  VsDriverInput.b_SwerveGoalAutoCenter          = LeCont_b_Driver1ButtonA;
  VsDriverInput.b_SwerveRotateTo0               = LeCont_b_Driver1ButtonX;
  VsDriverInput.b_SwerveRotateTo90              = LeCont_b_Driver1ButtonY;
  VsDriverInput.b_CameraLight                   = LeCont_b_Driver1ButtonRB;                      //Controller 1, X button (3), when held, turns on the camera light
  VsDriverInput.b_AutoIntake                    = LeCont_b_Driver1ButtonB;
  VsDriverInput.b_VisionDriverModeOverride      = LeCont_b_Driver1ButtonLB;
   
  L_AxisTotal = (fabs(VsDriverInput.pct_SwerveStrafe) + fabs(VsDriverInput.deg_SwerveRotate) + fabs(VsDriverInput.v_SwerveSpeed));
  
  if (L_AxisTotal > 0)
    {
    L_JoystickActive = true;
    }

  VsDriverInput.b_JoystickActive = L_JoystickActive;

  if (LeCont_Deg_Driver2POV == 0)
    {
    L_LiftCmndDirection = E_LiftCmndUp;
    }
  else if (LeCont_Deg_Driver2POV == 180)
    {
    L_LiftCmndDirection = E_LiftCmndDown;
    }
  else if (LeCont_Deg_Driver2POV == 90)
    {
    L_LiftCmndDirection = E_LiftCmndForward;
    }
  else if (LeCont_Deg_Driver2POV == 270)
    {
    L_LiftCmndDirection = E_LiftCmndBack;
    }
  else
    {
    L_LiftCmndDirection = E_LiftCmndNone;
    }

  VsDriverInput.e_LiftCmndDirection = L_LiftCmndDirection;

  if (LeCont_Deg_Driver1POV == 270)
    {
    L_e_TurretCmndDirection = E_TurrentCmndLeft;
    }
  else if (LeCont_Deg_Driver1POV == 90)
    {
    L_e_TurretCmndDirection = E_TurrentCmndRight;
    }
  else
    {
    L_e_TurretCmndDirection = E_TurrentCmndNone;
    }

  

   VsDriverInput.e_TurretCmndDirection = L_e_TurretCmndDirection;
  }


/******************************************************************************
 * Function:     Joystick1_robot_mapping
 *
 * Description:  Captures and maps driver inputs from controller 1.
 ******************************************************************************/
void Joystick1_robot_mapping(bool    LeCont_b_Driver1ButtonBack,
                             bool    LeCont_b_Driver1ButtonStart,
                             double  LeCont_Cmd_Driver1LeftAxisY,
                             double  LeCont_Cmd_Driver1LeftAxisX,
                             double  LeCont_Cmd_Driver1RightAxisX,
                             double  LeCont_Cmd_Driver1LeftTriggerAxis,
                             bool    LeCont_b_Driver1ButtonA,
                             bool    LeCont_b_Driver1ButtonX,
                             bool    LeCont_b_Driver1ButtonY,
                             bool    LeCont_b_Driver1ButtonRB,
                             bool    LeCont_b_Driver1ButtonB,
                             bool    LeCont_b_Driver1ButtonLB,
                             int     LeCont_Deg_Driver1POV)
  {
  double                L_AxisTotal             = 0;
  bool                  L_JoystickActive        = false;
  T_TurretCmndDirection L_e_TurretCmndDirection = E_TurrentCmndNone;

  VsDriverInput.b_ZeroGyro                      = (LeCont_b_Driver1ButtonBack || LeCont_b_Driver1ButtonStart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  VsDriverInput.pct_SwerveForwardBack           = ScaleJoystickAxis(LeCont_Cmd_Driver1LeftAxisY);  // Scale the axis, also used for debouncing
  VsDriverInput.pct_SwerveStrafe                = ScaleJoystickAxis(LeCont_Cmd_Driver1LeftAxisX);        // Scale the axis, also used for debouncing
  VsDriverInput.deg_SwerveRotate                = ScaleJoystickAxis(LeCont_Cmd_Driver1RightAxisX);      // Scale the axis, also used for debouncing
  VsDriverInput.v_SwerveSpeed                   = ScaleJoystickAxis(LeCont_Cmd_Driver1LeftTriggerAxis);  // Scale the axis, also used for debouncing
  VsDriverInput.b_SwerveGoalAutoCenter          = LeCont_b_Driver1ButtonA;
  VsDriverInput.b_SwerveRotateTo0               = LeCont_b_Driver1ButtonX;
  VsDriverInput.b_SwerveRotateTo90              = LeCont_b_Driver1ButtonY;
  VsDriverInput.b_CameraLight                   = LeCont_b_Driver1ButtonRB;                      //Controller 1, X button (3), when held, turns on the camera light
  VsDriverInput.b_AutoIntake                    = LeCont_b_Driver1ButtonB;
  VsDriverInput.b_VisionDriverModeOverride      = LeCont_b_Driver1ButtonLB;
   
  L_AxisTotal = (fabs(VsDriverInput.pct_SwerveStrafe) + fabs(VsDriverInput.deg_SwerveRotate) + fabs(VsDriverInput.v_SwerveSpeed));
  
  if (L_AxisTotal > 0)
    {
    L_JoystickActive = true;
    }

  VsDriverInput.b_JoystickActive = L_JoystickActive;

  if (LeCont_Deg_Driver1POV == 270)
    {
    L_e_TurretCmndDirection = E_TurrentCmndLeft;
    }
  else if (LeCont_Deg_Driver1POV == 90)
    {
    L_e_TurretCmndDirection = E_TurrentCmndRight;
    }
  else
    {
    L_e_TurretCmndDirection = E_TurrentCmndNone;
    }

   VsDriverInput.e_TurretCmndDirection = L_e_TurretCmndDirection;
  }

/******************************************************************************
 * Function:     Joystick2_robot_mapping
 *
 * Description:  Captures and maps driver inputs from controller 2.
 ******************************************************************************/
void Joystick2_robot_mapping(bool    LeCont_b_Driver2ButtonA,
                             bool    LeCont_b_Driver2ButtonB,
                             bool    LeCont_b_Driver2ButtonRB,
                             bool    LeCont_b_Driver2ButtonLB,
                             bool    LeCont_b_Driver2ButtonStart,
                             bool    LeCont_b_Driver2ButtonX,
                             bool    LeCont_b_Driver2ButtonY,
                             double  LeCont_Cmd_Driver2LeftAxisY,
                             double  LeCont_Cmd_Driver2RightAxisY,
                             int     LeCont_Deg_Driver2POV,
                             bool    LeCont_b_Driver2ButtonBack)
  {
  TeLFT_e_LiftCmndDirection   L_LiftCmndDirection     = E_LiftCmndNone;
  T_TurretCmndDirection L_e_TurretCmndDirection = E_TurrentCmndNone;


  VsDriverInput.b_ElevatorUp                    = LeCont_b_Driver2ButtonA;      //Controller 2, A button (1), (robot.cpp) Elevator goes up
  VsDriverInput.b_ElevatorDown                  = LeCont_b_Driver2ButtonB;      //Controller 2, B button (2), (robot.cpp) Elevator goes down
  VsDriverInput.b_StopShooterAutoClimbResetGyro = LeCont_b_Driver2ButtonLB;     //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain, pauses auto climb and resets encoders in test mode
  VsDriverInput.b_AutoSetSpeedShooter           = LeCont_b_Driver2ButtonStart;  //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  VsDriverInput.pct_ManualShooterDesiredSpeed   = LeCont_Cmd_Driver2LeftAxisY;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  VsDriverInput.b_LiftControl                   = LeCont_b_Driver2ButtonRB;     //Controller 2, X button (3), (Lift.cpp) starts automated states machine
  VsDriverInput.b_IntakeIn                      = LeCont_b_Driver2ButtonX;      //Controller 2 (3), controlls the intake in on trigger pressed
  VsDriverInput.b_IntakeOut                     = LeCont_b_Driver2ButtonY;      //Controller 2 (4), controlls the intake out on trigger pressed

  if (LeCont_Deg_Driver2POV == 0)
    {
    L_LiftCmndDirection = E_LiftCmndUp;
    }
  else if (LeCont_Deg_Driver2POV == 180)
    {
    L_LiftCmndDirection = E_LiftCmndDown;
    }
  else if (LeCont_Deg_Driver2POV == 90)
    {
    L_LiftCmndDirection = E_LiftCmndForward;
    }
  else if (LeCont_Deg_Driver2POV == 270)
    {
    L_LiftCmndDirection = E_LiftCmndBack;
    }
  else
    {
    L_LiftCmndDirection = E_LiftCmndNone;
    }

  VsDriverInput.e_LiftCmndDirection = L_LiftCmndDirection;
  }