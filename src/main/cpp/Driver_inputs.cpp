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
void Joystick_robot_mapping(bool    L_Driver2_buttonA,
                            bool    L_Driver2_buttonB,
                            bool    L_Driver2_buttonRB,
                            bool    L_Driver2_buttonLB,
                            bool    L_Driver2_buttonstart,
                            bool    L_Driver1_buttonback,
                            bool    L_Driver1_buttonstart,
                            bool    L_Driver2_ButtonX,
                            bool    L_Driver2_ButtonY,
                            double  L_Driver2_left_Axis_y,
                            double  L_Driver2_right_Axis_y,
                            double  L_Driver1_left_Axis_y,
                            double  L_Driver1_left_Axis_x,
                            double  L_Driver1_right_Axis_x,
                            double  L_Driver1_left_trigger_Axis,
                            bool    L_Driver1_buttonA,
                            bool    L_Driver1_ButtonX,
                            bool    L_Driver1_ButtonY,
                            int     L_Driver2_POV,
                            bool    L_Driver1_buttonRB,
                            bool    L_Driver1_buttonB,
                            bool    L_Driver1_ButtonLB,
                            bool    L_Driver2_buttonback,
                            int     L_Driver1_POV)
  {
  double                L_AxisTotal             = 0;
  bool                  L_JoystickActive        = false;
  T_LiftCmndDirection   L_LiftCmndDirection     = E_LiftCmndNone;
  T_TurretCmndDirection L_e_TurretCmndDirection = E_TurrentCmndNone;


  VsDriverInput.b_ElevatorUp                    = L_Driver2_buttonA;                       //Controller 2, A button (1), (robot.cpp) Elevator goes up
  VsDriverInput.b_ElevatorDown                  = L_Driver2_buttonB;                     //Controller 2, B button (2), (robot.cpp) Elevator goes down
  VsDriverInput.b_ZeroGyro                      = (L_Driver1_buttonback || L_Driver1_buttonstart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  VsDriverInput.b_StopShooterAutoClimbResetGyro = L_Driver2_buttonLB;     //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain, pauses auto climb and resets encoders in test mode
  VsDriverInput.b_AutoSetSpeedShooter           = L_Driver2_buttonstart;         //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  VsDriverInput.pct_ManualShooterDesiredSpeed   = L_Driver2_left_Axis_y;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  VsDriverInput.b_LiftControl                   = L_Driver2_buttonRB;                     //Controller 2, X button (3), (Lift.cpp) starts automated states machine
  VsDriverInput.b_IntakeIn                      = L_Driver2_ButtonX;                         //Controller 2 (3), controlls the intake in on trigger pressed
  VsDriverInput.b_IntakeOut                     = L_Driver2_ButtonY;                         //Controller 2 (4), controlls the intake out on trigger pressed
  VsDriverInput.pct_SwerveForwardBack           = ScaleJoystickAxis(L_Driver1_left_Axis_y);  // Scale the axis, also used for debouncing
  VsDriverInput.pct_SwerveStrafe                = ScaleJoystickAxis(L_Driver1_left_Axis_x);        // Scale the axis, also used for debouncing
  VsDriverInput.deg_SwerveRotate                = ScaleJoystickAxis(L_Driver1_right_Axis_x);      // Scale the axis, also used for debouncing
  VsDriverInput.v_SwerveSpeed                   = ScaleJoystickAxis(L_Driver1_left_trigger_Axis);  // Scale the axis, also used for debouncing
  VsDriverInput.b_SwerveGoalAutoCenter          = L_Driver1_buttonA;
  VsDriverInput.b_SwerveRotateTo0               = L_Driver1_ButtonX;
  VsDriverInput.b_SwerveRotateTo90              = L_Driver1_ButtonY;
  VsDriverInput.b_CameraLight                   = L_Driver1_buttonRB;                      //Controller 1, X button (3), when held, turns on the camera light
  VsDriverInput.b_AutoIntake                    = L_Driver1_buttonB;
  VsDriverInput.b_VisionDriverModeOverride      = L_Driver1_ButtonLB;
   
  L_AxisTotal = (fabs(VsDriverInput.pct_SwerveStrafe) + fabs(VsDriverInput.deg_SwerveRotate) + fabs(VsDriverInput.v_SwerveSpeed));
  
  if (L_AxisTotal > 0)
    {
    L_JoystickActive = true;
    }

  VsDriverInput.b_JoystickActive = L_JoystickActive;

  if (L_Driver2_POV == 0)
    {
    L_LiftCmndDirection = E_LiftCmndUp;
    }
  else if (L_Driver2_POV == 180)
    {
    L_LiftCmndDirection = E_LiftCmndDown;
    }
  else if (L_Driver2_POV == 90)
    {
    L_LiftCmndDirection = E_LiftCmndForward;
    }
  else if (L_Driver2_POV == 270)
    {
    L_LiftCmndDirection = E_LiftCmndBack;
    }
  else
    {
    L_LiftCmndDirection = E_LiftCmndNone;
    }

  VsDriverInput.e_LiftCmndDirection = L_LiftCmndDirection;

  if (L_Driver1_POV == 270)
    {
    L_e_TurretCmndDirection = E_TurrentCmndLeft;
    }
  else if (L_Driver1_POV == 90)
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
void Joystick1_robot_mapping(bool    L_Driver1_buttonback,
                             bool    L_Driver1_buttonstart,
                             double  L_Driver1_left_Axis_y,
                             double  L_Driver1_left_Axis_x,
                             double  L_Driver1_right_Axis_x,
                             double  L_Driver1_left_trigger_Axis,
                             bool    L_Driver1_buttonA,
                             bool    L_Driver1_ButtonX,
                             bool    L_Driver1_ButtonY,
                             bool    L_Driver1_buttonRB,
                             bool    L_Driver1_buttonB,
                             bool    L_Driver1_ButtonLB,
                             int     L_Driver1_POV)
  {
  double                L_AxisTotal             = 0;
  bool                  L_JoystickActive        = false;
  T_TurretCmndDirection L_e_TurretCmndDirection = E_TurrentCmndNone;

  VsDriverInput.b_ZeroGyro                      = (L_Driver1_buttonback || L_Driver1_buttonstart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  VsDriverInput.pct_SwerveForwardBack           = ScaleJoystickAxis(L_Driver1_left_Axis_y);  // Scale the axis, also used for debouncing
  VsDriverInput.pct_SwerveStrafe                = ScaleJoystickAxis(L_Driver1_left_Axis_x);        // Scale the axis, also used for debouncing
  VsDriverInput.deg_SwerveRotate                = ScaleJoystickAxis(L_Driver1_right_Axis_x);      // Scale the axis, also used for debouncing
  VsDriverInput.v_SwerveSpeed                   = ScaleJoystickAxis(L_Driver1_left_trigger_Axis);  // Scale the axis, also used for debouncing
  VsDriverInput.b_SwerveGoalAutoCenter          = L_Driver1_buttonA;
  VsDriverInput.b_SwerveRotateTo0               = L_Driver1_ButtonX;
  VsDriverInput.b_SwerveRotateTo90              = L_Driver1_ButtonY;
  VsDriverInput.b_CameraLight                   = L_Driver1_buttonRB;                      //Controller 1, X button (3), when held, turns on the camera light
  VsDriverInput.b_AutoIntake                    = L_Driver1_buttonB;
  VsDriverInput.b_VisionDriverModeOverride      = L_Driver1_ButtonLB;
   
  L_AxisTotal = (fabs(VsDriverInput.pct_SwerveStrafe) + fabs(VsDriverInput.deg_SwerveRotate) + fabs(VsDriverInput.v_SwerveSpeed));
  
  if (L_AxisTotal > 0)
    {
    L_JoystickActive = true;
    }

  VsDriverInput.b_JoystickActive = L_JoystickActive;

  if (L_Driver1_POV == 270)
    {
    L_e_TurretCmndDirection = E_TurrentCmndLeft;
    }
  else if (L_Driver1_POV == 90)
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
void Joystick2_robot_mapping(bool    L_Driver2_buttonA,
                             bool    L_Driver2_buttonB,
                             bool    L_Driver2_buttonRB,
                             bool    L_Driver2_buttonLB,
                             bool    L_Driver2_buttonstart,
                             bool    L_Driver2_ButtonX,
                             bool    L_Driver2_ButtonY,
                             double  L_Driver2_left_Axis_y,
                             double  L_Driver2_right_Axis_y,
                             int     L_Driver2_POV,
                             bool    L_Driver2_buttonback)
  {
  T_LiftCmndDirection   L_LiftCmndDirection     = E_LiftCmndNone;
  T_TurretCmndDirection L_e_TurretCmndDirection = E_TurrentCmndNone;


  VsDriverInput.b_ElevatorUp                    = L_Driver2_buttonA;      //Controller 2, A button (1), (robot.cpp) Elevator goes up
  VsDriverInput.b_ElevatorDown                  = L_Driver2_buttonB;      //Controller 2, B button (2), (robot.cpp) Elevator goes down
  VsDriverInput.b_StopShooterAutoClimbResetGyro = L_Driver2_buttonLB;     //Controller 2 back button (7), (robot.cpp) Stops the shooter- pretty self-explain, pauses auto climb and resets encoders in test mode
  VsDriverInput.b_AutoSetSpeedShooter           = L_Driver2_buttonstart;  //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  VsDriverInput.pct_ManualShooterDesiredSpeed   = L_Driver2_left_Axis_y;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  VsDriverInput.b_LiftControl                   = L_Driver2_buttonRB;     //Controller 2, X button (3), (Lift.cpp) starts automated states machine
  VsDriverInput.b_IntakeIn                      = L_Driver2_ButtonX;      //Controller 2 (3), controlls the intake in on trigger pressed
  VsDriverInput.b_IntakeOut                     = L_Driver2_ButtonY;      //Controller 2 (4), controlls the intake out on trigger pressed

  if (L_Driver2_POV == 0)
    {
    L_LiftCmndDirection = E_LiftCmndUp;
    }
  else if (L_Driver2_POV == 180)
    {
    L_LiftCmndDirection = E_LiftCmndDown;
    }
  else if (L_Driver2_POV == 90)
    {
    L_LiftCmndDirection = E_LiftCmndForward;
    }
  else if (L_Driver2_POV == 270)
    {
    L_LiftCmndDirection = E_LiftCmndBack;
    }
  else
    {
    L_LiftCmndDirection = E_LiftCmndNone;
    }

  VsDriverInput.e_LiftCmndDirection = L_LiftCmndDirection;
  }