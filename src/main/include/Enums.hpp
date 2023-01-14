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


typedef enum T_RoboShooter
{
  E_rightShooter,
  E_leftShooter,
  E_RoboShooter
} T_RoboShooter;


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


typedef enum T_AutoTargetStates
{
  E_NotActive, //not doing anything
  E_TargetFoundRotateBot, //target locked
  E_RollerSpinUp, //we movin
  E_MoveBallsToRollers, // get ready
  E_AutoTargetStatesSz // 
} T_AutoTargetStates;

typedef enum T_LauncherStates
{
  E_LauncherNotActive,
  E_LauncherAutoTargetActive,
  E_LauncherManualActive,
} T_LauncherStates;

typedef enum T_CameraLightStatus
{
  E_LightTurnedOff,
  E_LightOnWaitingForTarget,
  E_LightOnTargetingReady,
  E_LightForcedOffDueToOvertime
} T_CameraLightStatus;

typedef enum T_LiftCmndDirection
{
  E_LiftCmndNone,
  E_LiftCmndUp,
  E_LiftCmndDown,
  E_LiftCmndForward,
  E_LiftCmndBack
} T_LiftCmndDirection;

typedef enum T_TurretCmndDirection
{
  E_TurrentCmndNone,
  E_TurrentCmndLeft,
  E_TurrentCmndRight
} T_TurretCmndDirection;

typedef enum T_LED_LightCmnd
{
  E_LED_Red,
  E_LED_Blue,
  E_LED_Green,
  E_LED_Orange
} T_LED_LightCmnd;

typedef enum T_Lift_State
{
  E_S0_BEGONE,
  E_S2_lift_down_YD,
  E_S3_move_forward_XD,
  E_S4_stretch_up_YD,
  E_S5_more_forward_XD,
  E_S6_lift_up_more_YD,
  E_S7_move_back_XD,
  E_S8_more_down_some_YD,
  E_S9_back_rest_XD,
  E_S10_final_YD,
  E_S11_final_OWO,
  E_Lift_State_Sz
} T_Lift_State;

typedef enum T_Lift_Iteration
{
  E_LiftIteration1,
  E_LiftIteration2,
  E_LiftIterationSz
} T_Lift_Iteration;

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

typedef enum T_ADAS_UT_UpperTarget
{
  E_ADAS_UT_Disabled,
  E_ADAS_UT_CameraLightOn,
  E_ADAS_UT_AutoCenter,
  E_ADAS_UT_LauncherSpeed,
  E_ADAS_UT_ElevatorControl
} T_ADAS_UT_UpperTarget;

typedef enum T_ADAS_ActiveFeature
{
  E_ADAS_Disabled,
  E_ADAS_UT_AutoUpperTarget,
  E_ADAS_BT_AutoBallTarget,
  E_ADAS_DM_BlindLaunch,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_ReverseAndIntake,
  E_ADAS_DM_Rotate180,
  E_ADAS_DM_RotateFieldOriented,
  E_ADAS_DM_PathFollower
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
  E_ADAS_AutonDriveAndShootAuto3       // Drive into preplaced ball, intake, rotate 180*, shoot 2 balls, pickup 3rd and shoot
} T_ADAS_ActiveAutonFeature;

typedef enum T_MotorControlType
{
  E_MotorControlDisabled,
  E_MotorControlPctCmnd,
  E_MotorControlPosition,
  E_MotorControlSpeed
} T_MotorControlType;

typedef enum TeTurretInitialization
{
  E_TurretInitDisabled,
  E_TurretInitOL_Right,
  E_TurretInitOL_Left,
  E_TurretInitLimitSwitch,
  E_TurretInitRotateToZeroPosition,
  E_TurretInitComplete,
  E_TurretInitFaultDetected
} TeTurretInitialization;

typedef enum TeTurretState
{
  E_TurretDisabled,
  E_TurretInitialization,
  E_TurretCameraControl
} TeTurretState;





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
  bool                  b_LiftControl;
  bool                  b_ZeroGyro;
  bool                  b_StopShooterAutoClimbResetGyro;
  bool                  b_AutoSetSpeedShooter;
  bool                  b_ElevatorUp;
  bool                  b_ElevatorDown;
  double                pct_ManualShooterDesiredSpeed;
  bool                  b_IntakeIn;
  bool                  b_IntakeOut;
  double                pct_SwerveForwardBack;
  double                pct_SwerveStrafe;
  double                deg_SwerveRotate;
  double                v_SwerveSpeed;
  bool                  b_SwerveGoalAutoCenter;
  bool                  b_SwerveRotateTo0;
  bool                  b_SwerveRotateTo90;
  bool                  b_LiftYD_Up;
  bool                  b_LiftYD_Down;
  T_LiftCmndDirection   e_LiftCmndDirection;
  bool                  b_CameraLight;
  bool                  b_AutoIntake;
  bool                  b_JoystickActive;
  bool                  b_VisionDriverModeOverride;
  bool                  b_RobotFieldOrientedReq;
  T_TurretCmndDirection e_TurretCmndDirection;
};

struct TsRobotMotorCmnd
{
  T_MotorControlType    e_TurretControlType;
  double                pct_TurretCmnd;
  double                deg_TurretCmnd;
};

#endif
