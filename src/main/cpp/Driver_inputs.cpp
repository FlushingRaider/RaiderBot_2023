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

  VsCONT_s_DriverInput.b_ZeroGyro                      = (LeCONT_b_Driver1ButtonBack || LeCONT_b_Driver1ButtonStart);     //Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro  
  VsCONT_s_DriverInput.pct_SwerveForwardBack           = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftAxisY);  // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.pct_SwerveStrafe                = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftAxisX);        // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.deg_SwerveRotate                = ScaleJoystickAxis(LeCONT_Cmd_Driver1RightAxisX);      // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.v_SwerveSpeed                   = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftTriggerAxis);  // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.b_AutoBalance                   = LeCONT_b_Driver1ButtonA;
 // VsCONT_s_DriverInput.b_SwerveRotateTo0             = LeCONT_b_Driver1ButtonX;
 // VsCONT_s_DriverInput.b_SwerveRotateTo90            = LeCONT_b_Driver1ButtonY;
  VsCONT_s_DriverInput.b_CubeAlign                     = LeCONT_b_Driver1ButtonRB;    //Aligns the robot to score a cone
//  VsCONT_s_DriverInput.b_AutoIntake                  = LeCONT_b_Driver1ButtonB;
  VsCONT_s_DriverInput.b_ConeAlign                     = LeCONT_b_Driver1ButtonLB;   //Aligns the robot to score a cone
   
  LeCONT_Pct_AxisTotal = (fabs(VsCONT_s_DriverInput.pct_SwerveStrafe) + fabs(VsCONT_s_DriverInput.deg_SwerveRotate) + fabs(VsCONT_s_DriverInput.v_SwerveSpeed));
  
  if (LeCONT_Pct_AxisTotal > 0)
    {
    LeCONT_b_JoystickActive = true;
    }

  VsCONT_s_DriverInput.b_JoystickActive = LeCONT_b_JoystickActive;
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
                             double  LeCONT_Pct_Driver2LeftAxisY,
                             double  LeCONT_Pct_Driver2RightAxisX,
                             int     LeCONT_Deg_Driver2POV,
                             bool    LeCONT_b_Driver2ButtonBack,
                             double  LeCont_Pct_Driver2AxisRB,
                             double  LeCont_Pct_Driver2AxisLB)
  {
  double LeCONT_Pct_TurretTestCmnd = 0.0;
  double LeCONT_Pct_IntakeRollerTestCmnd = 0.0;

  VsCONT_s_DriverInput.b_MainIntakeOut                  = LeCONT_b_Driver2ButtonA;      //Controller 2, A button Will be used to bring intake out COMPETION BUTTON
  VsCONT_s_DriverInput.b_DrivingPosition                = LeCONT_b_Driver2ButtonB;     //Controller 2, B button Will be used to bring Everything into their position for when the robot is moving COMPETION BUTTON
  VsCONT_s_DriverInput.b_IntakeArmOutTest               = LeCONT_b_Driver2ButtonY;      //Controller 2, Y button (2), (robot.cpp) intake out TEST BUTTON
  VsCONT_s_DriverInput.b_IntakeArmIn                    = LeCONT_b_Driver2ButtonA;      //Controller 2, A button (1), (robot.cpp) intake in TEST BUTTON
  VsCONT_s_DriverInput.b_HighPositionCube               = LeCONT_b_Driver2ButtonY;      //Controller 2, Y button, Tells robot that we are scoring the gamepiece high COMPETION BUTTON
  VsCONT_s_DriverInput.b_IntakeRollersTest              = LeCONT_b_Driver2ButtonX;     //Controller 2 Tests the intake rollers.  For test only.
  VsCONT_s_DriverInput.b_LowPositionCube                = LeCONT_b_Driver2ButtonX;
  VsCONT_s_DriverInput.b_ResetManipulatorEnocders       = LeCONT_b_Driver2ButtonStart;  //controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  VsCONT_s_DriverInput.b_DropGamePiece                  = LeCONT_b_Driver2ButtonStart;
  VsCONT_s_DriverInput.Pct_WristTest                    = LeCONT_Pct_Driver2RightAxisX;  //Controller 2, left axis, uses y axis (1), (robot.cpp) sets desired speed for the shooter moter
  VsCONT_s_DriverInput.b_VisionButton                   = LeCONT_b_Driver2ButtonLB;    // Vision button for Carson uses, might keep for comp might end up being just for testing
  VsCONT_s_DriverInput.b_ArmDown                        = LeCONT_b_Driver2ButtonRB;   // This will bring the arm down to pickup game pieces behind the robot

  if (LeCONT_b_Driver2ButtonRB == true)
    {
    LeCONT_Pct_TurretTestCmnd += 1.0;
    }

  if (LeCONT_b_Driver2ButtonLB == true)
    {
    LeCONT_Pct_TurretTestCmnd += -1.0;
    }

  VsCONT_s_DriverInput.Pct_TurretTest = LeCONT_Pct_TurretTestCmnd;

  if (LeCont_Pct_Driver2AxisRB > 0.1) // Deadband
    {
    LeCONT_Pct_IntakeRollerTestCmnd += -LeCont_Pct_Driver2AxisRB;
    }

  if (LeCont_Pct_Driver2AxisLB > 0.1) // Deadband
    {
    LeCONT_Pct_IntakeRollerTestCmnd += LeCont_Pct_Driver2AxisLB;
    }

  VsCONT_s_DriverInput.pct_IntakeRollerTest = LeCONT_Pct_IntakeRollerTestCmnd;


  VsCONT_s_DriverInput.Pct_ArmPivotTest = 0.0;
  VsCONT_s_DriverInput.Pct_LinearSlideTest = 0.0;
  VsCONT_s_DriverInput.b_HighPositionCone = false;
  VsCONT_s_DriverInput.b_LowPositionCone = false;

  if (LeCONT_Deg_Driver2POV == 0)
    {
    VsCONT_s_DriverInput.Pct_ArmPivotTest = 1.0;
    VsCONT_s_DriverInput.b_HighPositionCone = true;
    }
  else if (LeCONT_Deg_Driver2POV == 180)
    {
    VsCONT_s_DriverInput.Pct_ArmPivotTest = -1.0;
    VsCONT_s_DriverInput.b_LowPositionCone = true;
    }
  else if (LeCONT_Deg_Driver2POV == 270)
    {
    VsCONT_s_DriverInput.Pct_LinearSlideTest = 1.0;
    }
  else if (LeCONT_Deg_Driver2POV == 90)
    {
    VsCONT_s_DriverInput.Pct_LinearSlideTest = -1.0;
    }
  }