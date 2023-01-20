/*
  Driver_inputs.hpp

   Created on: Feb 05, 2022
   Author: Lauren and Chloe uwu

  Function that maps the driver inputs to the robot controls. 
 */

 extern bool                VCmdLFT_b_DriverLiftControl;
 extern bool                VCmdGRY_b_DriverZeroGyro;
 extern bool                V_Driver_StopShooterAutoClimbResetGyro;/*Scrap? Shooter or Climber?*/
 extern bool                V_Driver_auto_setspeed_shooter; /*Scrap, shooter isnt used in this game*/
 extern bool                VeElev_b_DriverElevatorUp;
 extern bool                VeElev_b_DriverElevatorDown;
 extern double              V_Driver_manual_shooter_desired_speed; /*Scrap, Shooter isnt used in this game*/
 extern bool                VeInt_b_IntakeIn;/*Reuse for succ machine*/
 extern bool                VeInt_b_DriverIntakeOut;/*Reuse for succ Machine*/
 extern double              VeSD_Pct_SwerveForwardBack;
 extern double              VeSD_Pct_SwerveStrafe;
 extern double              VeSD_Pct_SwerveRotate;
 extern double              VeSD_RPM_SwerveSpeed;
 extern bool                VeSD_b_SwerveGoalAutoCenter;
 extern bool                VeSD_b_SwerveRotateTo0;
 extern bool                VeSD_b_SwerveRotateTo90;
 extern bool                VeLFT_b_DriverYDUp;
 extern bool                VeLFT_b_DriverYDDown;
 extern bool                VeLFT_b_DriverLiftCommandDirection;
 extern bool                VeLC_b_DriverCameraLight;
 extern bool                VeInt_b_DriverAutoIntake;
 extern bool                VeCont_b_DriverJoystickActive;
 extern bool                VeVis_b_VisionDriverModeOverride;
 extern bool                V_Driver_RobotFieldOrientedReq;/*Scrap, Because Robot Oriented Sux*/
 extern RobotUserInput    VsDriverInput;
 

void Joystick_robot_mapping(bool    LeCont_b_Driver2ButtonA, //Controller 2, A button (1), (robot.cpp) Elevator goes up
                            bool    LeCont_b_Driver2ButtonB, //Controller 2, B button (2), (robot.cpp) Elevator goes down
                            bool    LeCont_b_Driver2ButtonRB, //Controller 2, X button (3)
                            bool    LeCont_b_Driver2ButtonLB, //Controller 2 back button (7)
                            bool    LeCont_b_Driver2ButtonStart, //controller 2 start button (8)
                            bool    LeCont_b_Driver1ButtonBack, //Controller 1, Back button (7)
                            bool    LeCont_b_Driver1ButtonStart, //Controller 1, start button (8)
                            bool    LeCont_b_Driver2ButtonX,  //Controller 2 (3), controls the intake in on button pressed 
                            bool    LeCont_b_Driver2ButtonY, // Controller 2 (4), controls the intake out on button pressed (inverse intake)
                            double  LeCont_Cmd_Driver2LeftAxisY, //Controller 2, left axis, uses y (1) 
                            double  LeCont_Cmd_Driver2RightAxisY, //Controller 2,right axis, uses y (5)
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
                            int     LeCont_Deg_Driver1POV);
 
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
                             int     LeCont_Deg_Driver1POV);

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
                             bool    LeCont_b_Driver2ButtonBack);