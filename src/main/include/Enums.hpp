/*
  Enums.hpp

   Created on: Jan 3, 2020
   Author: 5561
 */

#pragma once

#ifndef ENUMS
#define ENUMS

typedef enum T_RobotCorner
{
  E_FrontLeft,
  E_FrontRight,
  E_RearLeft,
  E_RearRight,
  E_RobotCornerSz
} T_RobotCorner;


typedef enum T_PID_Cal
{
  E_P_Gx,
  E_I_Gx,
  E_D_Gx,
  E_P_Ul,
  E_P_Ll,
  E_I_Ul,
  E_I_Ll,
  E_D_Ul,
  E_D_Ll,
  E_Max_Ul,
  E_Max_Ll,
  E_PID_CalSz
} T_PID_Cal;


typedef enum T_PID_SparkMaxCal
{
  E_kP,
  E_kI,
  E_kD,
  E_kIz,
  E_kFF,
  E_kMaxOutput,
  E_kMinOutput,
  E_kMaxVel,
  E_kMinVel,
  E_kMaxAcc,
  E_kAllErr,
  E_PID_SparkMaxCalSz
} T_PID_SparkMaxCal;


typedef enum T_LauncherStates
{
  E_LauncherNotActive,
  E_LauncherAutoTargetActive,
  E_LauncherManualActive,
} T_LauncherStates;

typedef enum T_CameraLightStatus
{
  VeLC_e_LightTurnedOff,
  E_LightOnWaitingForTarget,
  E_LightOnTargetingReady,
  E_LightForcedOffDueToOvertime
} T_CameraLightStatus;

typedef enum T_LED_LightCmnd
{
  E_LED_Red,
  E_LED_Blue,
  E_LED_Green,
  E_LED_Orange
} T_LED_LightCmnd;

/* TeMAN_ManipulatorStates: States of the manipulator for the 2023 game. */
typedef enum TeMAN_ManipulatorStates
{
 E_MAN_Init,
 E_MAN_Driving,
 E_MAN_PositioningHighCube,
 E_MAN_PositioningHighCone,
 E_MAN_PositioningLowCube,
 E_MAN_PositioningLowCone,
 E_MAN_MidTransition,
 E_MAN_MainIntake,
 E_MAN_FloorIntake,
 E_MAN_State_Sz
} TeMAN_ManipulatorStates;
 

typedef enum T_RobotState
{
  E_Init,
  E_Auton,
  E_Teleop
} T_RobotState;

typedef enum T_CameraNumber
{
  E_Cam1,
  E_Cam2,
  E_CamSz
} T_CameraNumber;

typedef enum T_CameraLocation
{
  E_CamTop,
  E_CamBottom,
  E_CamLocSz
} T_CameraLocation;

typedef enum T_ADAS_BT_BallTarget /* aka GetDaBalls */
{
  E_ADAS_BT_Disabled,
  E_ADAS_BT_CameraLightOn,
  E_ADAS_BT_AutoCenter,
  E_ADAS_BT_IntakeAndRun
} T_ADAS_BT_BallTarget;

typedef enum T_ADAS_ActiveFeature
{
  E_ADAS_Disabled,
  E_ADAS_DM_CubeAlign,
  E_ADAS_DM_ConeAlign,
  E_ADAS_DM_AutoBalance,
  
  E_ADAS_MoveToTag,
  E_ADAS_BT_AutoBallTarget,
  E_ADAS_DM_BlindLaunch,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_ReverseAndIntake,
  E_ADAS_DM_Rotate180,
  E_ADAS_DM_RotateFieldOriented,
  E_ADAS_DM_PathFollower,
  //Added for 2023
  E_ADAS_AutonDeployCube,
  E_ADAS_AutonDeployCone,
  E_ADAS_AutonDrivePath,
  E_ADAS_AutonRotate,
  E_ADAS_AutonDrive
} T_ADAS_ActiveFeature;

typedef enum T_ADAS_Auton1
{
  E_ADAS_Auton_DM_PF_1,
  E_ADAS_Auton_BT_2,
  E_ADAS_Auton_DM_PF_3,
  E_ADAS_Auton_UT_4,
  E_ADAS_Auton_DM_PF_5,
  E_ADAS_Auton_BT_6,
  E_ADAS_Auton_DM_Rotate_7,
  E_ADAS_Auton_UT_8
} T_ADAS_Auton1;

typedef enum T_ADAS_ActiveAutonFeature
{
  E_ADAS_AutonDisabled,
  E_ADAS_AutonDriveAndShootBlind1,     // Shoot preloaded ball, drive straight, robot oriented
  E_ADAS_AutonDriveAndShootBlind2,     // Drive into preplaced ball, intake, rotate 180*, shoot 2 balls
  E_ADAS_AutonDriveAndShootAuto2,      // Drive into preplaced ball, intake, rotate 180*, shoot 2 balls
  E_ADAS_AutonDriveAndShootAuto3,// Drive into preplaced ball, intake, rotate 180*, shoot 2 balls, pickup 3rd and shoot
  E_ADAS_AutonDrivePath1      
} T_ADAS_ActiveAutonFeature;

typedef enum TeMAN_e_ManipulatorActuator
{
  E_MAN_Turret,
  E_MAN_ArmPivot,
  E_MAN_LinearSlide,
  E_MAN_Wrist,
  E_MAN_Gripper,
  E_MAN_IntakeRollers,
  E_MAN_IntakeArm,
  E_MAN_Sz
} TeMAN_e_ManipulatorActuator;

typedef enum T_MotorControlType
{
  E_MotorControlDisabled,
  E_MotorControlPctCmnd,
  E_MotorControlPosition,
  E_MotorControlSpeed,
  E_MotorExtend,
  E_MotorRetract
} T_MotorControlType;

struct TsMAN_Sensor 
{
  bool   b_TurretZero;
  double Deg_Turret; // Position of the turret rotation
  double Deg_ArmPivot; // Posistion of the Arm Angle
  double RPM_IntakeRollers; // Speed of the intake rollers
  double In_LinearSlide;
  bool   b_IntakeArmExtended;
  double RPM_Gripper; // Speed of the gripper
  double Deg_Wrist;  // Actual position of the wrist.
};

struct TsRobotSensor 
{
  bool b_TurretZero;
  bool b_XD_LimitDetected;
  bool b_XY_LimitDetected;
  bool b_BallDetectedLower;
  bool b_BallDetectedUpper;
};

struct RobotUserInput
{
  bool                  b_ZeroGyro;
  double                pct_SwerveForwardBack;
  double                pct_SwerveStrafe;
  double                deg_SwerveRotate;
  double                v_SwerveSpeed;
  bool                  b_SwerveGoalAutoCenter;
  bool                  b_SwerveRotateTo0;
  bool                  b_SwerveRotateTo90;
  bool                  b_CameraLight;
  bool                  b_AutoIntake;
  bool                  b_JoystickActive;
  bool                  b_VisionDriverModeOverride;
  bool                  b_IntakeRollersTest; //21
  bool                  b_ResetManipulatorEnocders; // 21
  bool                  b_IntakeArmIn;  // 21
  bool                  b_IntakeArmOutTest;
  double                Pct_TurretTest;
  double                Pct_WristTest;
  double                Pct_ArmPivotTest;
  double                Pct_LinearSlideTest;
  double                pct_IntakeRollerTest;
  bool                  b_MainIntakeOut;
  bool                  b_DrivingPosition;
  bool                  b_LowPositionCube;
  bool                  b_HighPositionCube;
  bool                  b_LowPositionCone;
  bool                  b_HighPositionCone;
  bool                  b_DropGamePiece;
  bool                  b_AutoBalance;
  bool                  b_CubeAlign;
  bool                  b_ConeAlign;
  bool                  b_VisionButton;
  bool                  b_ArmDown;  //back pickup
};

struct TeMAN_MotorControl
{
  T_MotorControlType    e_MotorControlType[E_MAN_Sz];
  double                k_MotorCmnd[E_MAN_Sz];
  double                k_MotorRampRate[E_MAN_Sz];
  double                k_MotorTestValue[E_MAN_Sz];
  double                k_MotorTestPower[E_MAN_Sz];
};

struct TeADAS_Controls
{
  TeMAN_ManipulatorStates e_MAN_State;
  bool                    b_MAN_DropObject;
  double                  Pct_SD_FwdRev;
  double                  Pct_SD_Strafe;
  double                  Pct_SD_Rotate;
  double                  b_SD_RobotOriented;
};

#endif
