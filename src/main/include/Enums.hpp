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
 E_MAN_MainIntake,
 E_MAN_FloorConeIntake,
 E_MAN_MidCubeIntake,
 E_MAN_MidConeIntake,
 E_MAN_HighCubeDrop,
 E_MAN_LowCubeDrop,
 E_MAN_HighConeDrop,
 E_MAN_LowConeDrop,
 E_MAN_State_Sz
} TeMAN_ManipulatorStates;

// /* TeMAN_ManipulatorStates: States of the manipulator for the 2023 game. */
// typedef enum TeMAN_ManipulatorStates
// {
//  E_MAN_Init,
//  E_MAN_Driving,
//  E_MAN_BackHighCube,
//  E_MAN_BackHighCone,
//  E_MAN_BackLowCube,
//  E_MAN_BackLowCone,
//  E_MAN_MidTransition,
//  E_MAN_MainIntake,
//  E_MAN_FloorIntake,
//  E_MAN_MidIntake,
//  E_MAN_FrontHighCube,
//  E_MAN_FrontLowCube,
//  E_MAN_State_Sz
// } TeMAN_ManipulatorStates;

/* TeADAS_AutonManipulatorStates: States of the manipulator for the 2023 game. */
typedef enum TeADAS_AutonManipulatorStates
{
 E_ADAS_MAN_Driving,
 E_ADAS_MAN_MidDropPosition,
 E_ADAS_MAN_MidDropOff,
 E_ADAS_MAN_State_Sz
} TeADAS_AutonManipulatorStates;

/* TeADAS_Auton1_DeployMAN_DriveOnStation: States of the manipulator for the 2023 game. */
typedef enum TeADAS_Auton1_DeployMAN_DriveOnStation
{
 E_ADAS_Auton1_DriveRevDplyMAN,
 E_ADAS_Auton1_StopDplyCube,
 E_ADAS_Auton1_FwdMountStationRetractMAN,
 E_ADAS_Auton1_AutoBal,
 E_ADAS_Auton1_Sz
} TeADAS_Auton1_DeployMAN_DriveOnStation;

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

typedef enum T_ADAS_ActiveFeature // These are the sub features in ADAS.  These can be called in teleop and/or auton
{
  E_ADAS_Disabled,
  E_ADAS_DM_CubeAlign,
  E_ADAS_DM_ConeAlign,
  E_ADAS_DM_AutoBalance,
  E_ADAS_DM_MountDismountRamp,
  E_ADAS_DM_MountRamp,
  E_ADAS_DM_DriveRevDeployArm,
  E_ADAS_DM_StopDeployCube,
  
  E_ADAS_MoveOffsetTag,
  E_ADAS_MoveGlobalTag,
  E_ADAS_BT_AutoBallTarget,
  E_ADAS_DM_BlindLaunch,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_DriveStraightFar,
  E_ADAS_DM_DriveRevStraight,
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

typedef enum T_ADAS_ActiveAutonFeature  // This is the high level feature, called by the driver station
{
  E_ADAS_AutonDisabled,
  E_ADAS_AutonDriveStraight,
  E_ADAS_AutonDropCubeDriveFwd,
  E_ADAS_AutonDriveOverRampAutoBalV2,
  E_ADAS_AutonDeliverCubeDriveOnRampAutoBal,
  E_ADAS_AutonDrivePath1
} T_ADAS_ActiveAutonFeature;


typedef enum TeADAS_DM_DriveOverStation  // This is the states of the low level feature "Drive over station"
{
  E_ADAS_DM_DriveOS_FwdFlat1,
  E_ADAS_DM_DriveOS_FwdRampUp,
  E_ADAS_DM_DriveOS_FwdRampDwn,
  E_ADAS_DM_DriveOS_RevRampUp,
  E_ADAS_DM_DriveOS_Complete
} TeADAS_DM_DriveOverStation;

typedef enum TeMAN_e_ManipulatorActuator
{
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
  double Deg_ArmPivot; // Posistion of the Arm Angle
  double RPM_IntakeRollers; // Speed of the intake rollers
  double In_LinearSlide;
  bool   b_IntakeArmExtended;
  double RPM_Gripper; // Speed of the gripper
  double Deg_Wrist;  // Actual position of the wrist.
  bool   b_GripperObjDetected;
};

struct TsRobotSensor 
{
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
  double                Pct_WristTest;
  double                Pct_ArmPivotTest;
  double                Pct_LinearSlideTest;
  double                pct_IntakeRollerTest;
  bool                  b_MainIntakeOut;
  bool                  b_MidIntakeOut;
  bool                  b_InitState;
  bool                  b_DrivingPosition;
  bool                  b_FrontHighCube;
  bool                  b_FrontLowCube;
  bool                  b_FrontHighCone;
  bool                  b_FrontLowCone;
  bool                  b_DropGamePieceSlow;
  bool                  b_DropGamePieceFast;
  bool                  b_AutoBalance;
  bool                  b_CubeAlign;
  bool                  b_ConeAlign;
  bool                  b_VisionButton;
  bool                  b_X_Mode;
};

struct TeMAN_MotorControl
{
  T_MotorControlType    e_MotorControlType[E_MAN_Sz];
  double                k_MotorCmnd[E_MAN_Sz];
  double                k_MotorRampRate[E_MAN_Sz];
  double                k_MotorTestValue[E_MAN_Sz];
  double                k_MotorTestPower[E_MAN_Sz];
};

#endif
