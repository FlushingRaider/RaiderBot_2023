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
  VeLC_e_LightTurnedOff,
  E_LightOnWaitingForTarget,
  E_LightOnTargetingReady,
  E_LightForcedOffDueToOvertime
} T_CameraLightStatus;

typedef enum T_Manipulator_CmndDirection
{
  E_LiftCmndNone,
  E_LiftCmndUp,
  E_LiftCmndDown,
  E_LiftCmndForward,
  E_LiftCmndBack
} T_Manipulator_CmndDirection;

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

typedef enum TeMAN_ManipulatorStates
{E_S0_Rest,
 E_S1_Intake,
 E_S2_TradeOff,
 E_S3_Swiper,
 E_S4_DrivingState,
 E_S5_Positioning,
 E_S6_DroppingTheLoot,
 E_Man_State_Sz
} TeMAN_ManipulatorStates;
 

// typedef enum T_Man_Iteration
// {
//   VeMAN_Cnt_ManIterationNew,
//   E_LiftIteration2,
//   E_LiftIterationSz
// } T_Man_Iteration;

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
  E_ADAS_DM_PathFollower,
  //Added for 2023
  E_ADAS_AutonDeployCube,
  E_ADAS_AutonDeployCone,
  E_ADAS_AutonDrivePath,
  E_ADAS_AutonRotate,

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
  E_ADAS_AutonDriveAndShootAuto3,       // Drive into preplaced ball, intake, rotate 180*, shoot 2 balls, pickup 3rd and shoot
    //Added for 2023

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
  E_MotorFwd,
  E_MotorRev
} T_MotorControlType;

struct TsMAN_Sensor 
{
  bool   b_TurretZero;
  double Deg_Turret; // Position of the turret rotation
  double Deg_ArmPivot; // Posistion of the Arm Angle
  double RPM_IntakeRollers; // Speed of the intake rollers
  double In_LinearSlide;
  bool   b_IntakeArmExtended;
  double Deg_Gripper; // Position of the gripper
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
  bool                  b_RobotFieldOrientedReq;  // ToDo: Remove
  T_TurretCmndDirection e_TurretCmndDirection;  // ToDo: Remove
  bool                  b_IntakeRollers; //21
  bool                  b_ResetManipulatorEnocders; // 21
  bool                  b_IntakeArmIn;  // 21
  bool                  b_IntakeArmOut; // 21
  double                pct_Turret; // 21
  double                pct_Wrist; // 21
  double                pct_ArmPivot; // 21
  double                pct_LinearSlide; // 21
  double                pct_Claw; // 21
};



struct TeMAN_MotorControl
{
  T_MotorControlType    e_MotorControlType[E_MAN_Sz];
  double                k_MotorCmnd[E_MAN_Sz];
  double                k_MotorRampRate[E_MAN_Sz];
  double                k_MotorTestValue[E_MAN_Sz];
  double                k_MotorTestPower[E_MAN_Sz];
};

struct TsRobotMotorCmnd
{
  T_MotorControlType    e_TurretControlType;
  double                pct_TurretCmnd;
  double                deg_TurretCmnd;
};

#endif
