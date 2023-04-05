/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls without vision (beta 02/26/2022)
 * - Climber active (beta 02/26/2022)
 *
 * */

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "Encoders.hpp"
#include "Gyro.hpp"
#include "IO_Sensors.hpp"
#include "Driver_inputs.hpp"
#include "Odometry.hpp"
#include "DriveControl.hpp"
#include "Manipulator.hpp"
#include "LightControl.hpp"
#include "VisionV2.hpp"
#include "ADAS.hpp"
#include "ADAS_DM.hpp"
#include "ADAS_MN.hpp"

T_RobotState VeROBO_e_RobotState = E_Init;
frc::DriverStation::Alliance VeROBO_e_AllianceColor = frc::DriverStation::Alliance::kInvalid;
double VeROBO_t_MatchTimeRemaining = 0;
bool VeROBO_b_TestState = false;

// bool FakeButton;

/******************************************************************************
 * Function:     RobotMotorCommands
 *
 * Description:  Contains the outputs for the motors.
 ******************************************************************************/
void Robot::RobotMotorCommands()
{
  bool LeROBO_b_IntakeArmExtend = false;
  // Motor output commands:
  // Swerve drive motors
  // Swerve stear motors
  if (VeDRC_b_DriveWheelsInPID == true)
  {
    m_frontLeftDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_FrontLeft], rev::CANSparkMax::ControlType::kVelocity); // rev::ControlType::kVelocity
    m_frontRightDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_FrontRight], rev::CANSparkMax::ControlType::kVelocity);
    m_rearLeftDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_RearLeft], rev::CANSparkMax::ControlType::kVelocity);
    m_rearRightDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_RearRight], rev::CANSparkMax::ControlType::kVelocity);
    m_frontLeftSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_RearRight]);
  }
  else
  {
    m_frontLeftDriveMotor.Set(0);
    m_frontRightDriveMotor.Set(0);
    m_rearLeftDriveMotor.Set(0);
    m_rearRightDriveMotor.Set(0);
    m_frontLeftSteerMotor.Set(0);
    m_frontRightSteerMotor.Set(0);
    m_rearLeftSteerMotor.Set(0);
    m_rearRightSteerMotor.Set(0);
  }

#ifdef CompBot
  m_ArmPivotPID.SetReference(VsMAN_s_Motors.k_MotorCmnd[E_MAN_ArmPivot], rev::ControlType::kPosition);
 // m_ArmPivot.Set(0.0);
  m_WristPID.SetReference(VsMAN_s_Motors.k_MotorCmnd[E_MAN_Wrist], rev::ControlType::kPosition);
  // m_GripperPID.SetReference(VsMAN_s_Motors.k_MotorCmnd[E_MAN_Gripper], rev::ControlType::kVelocity);
  m_Gripper.Set(VsMAN_s_Motors.k_MotorCmnd[E_MAN_Gripper]); // This puts the gripper into a power control setup, not speed/postion
  // m_IntakeRollersPID.SetReference(VsMAN_s_Motors.k_MotorCmnd[E_MAN_IntakeRollers], rev::ControlType::kVelocity);
  m_IntakeRollers.Set(VsMAN_s_Motors.k_MotorCmnd[E_MAN_IntakeRollers]);
  // m_TurretRotate.Set(ControlMode::PercentOutput, 0.0);
  m_LinearSlide.Set(ControlMode::PercentOutput, VsMAN_s_Motors.k_MotorCmnd[E_MAN_LinearSlide]);
  //m_LinearSlide.Set(ControlMode::PercentOutput, 0.0);

  if (VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] == E_MotorExtend)
  {
    LeROBO_b_IntakeArmExtend = true;
  }

  m_PCM_Valve.Set(LeROBO_b_IntakeArmExtend);
#endif
}

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
{
  VeROBO_e_RobotState = E_Init;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();
  VeROBO_t_MatchTimeRemaining = frc::Timer::GetMatchTime().value();
  VeROBO_b_TestState = false;

  // frc::CameraServer::StartAutomaticCapture();  // For connecting a single USB camera directly to RIO

  EncodersInitCommon(m_encoderFrontRightSteer,
                     m_encoderFrontLeftSteer,
                     m_encoderRearRightSteer,
                     m_encoderRearLeftSteer,
                     m_encoderFrontRightDrive,
                     m_encoderFrontLeftDrive,
                     m_encoderRearRightDrive,
                     m_encoderRearLeftDrive);
#ifdef CompBot
  EncodersInitComp(m_ArmPivotEncoder,
                   m_WristEncoder,
                   m_GripperEncoder,
                   m_IntakeRollersEncoder);

  bool CompressorEnable = m_pcmCompressor.Enabled();
#endif

  GyroInit();

  IO_SensorsInit();

  m_frontLeftSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_frontRightSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_rearLeftSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_rearRightSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);

  m_frontLeftSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontLeftDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontRightSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontRightDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearLeftSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearLeftDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearRightSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearRightDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

#ifdef CompBot
  m_ArmPivot.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Wrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Wrist.SetSmartCurrentLimit(KeMAN_A_ManipulatorNeoCurrentLim);
  m_Gripper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_Gripper.SetSmartCurrentLimit(KeMAN_A_ManipulatorNeoCurrentLimHigh);
  m_IntakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_IntakeRollers.SetSmartCurrentLimit(KeMAN_A_ManipulatorNeoCurrentLim);

  // m_WristforwardLimit.EnableLimitSwitch(false);
  m_WristreverseLimit.EnableLimitSwitch(false);

  m_LinearSlide.ConfigFactoryDefault();
  m_LinearSlide.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.SetSensorPhase(true);
  m_LinearSlide.SetInverted(true);
  // m_LinearSlide.SetSelectedSensorPosition(0);
  m_LinearSlide.ConfigNominalOutputForward(0, KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.ConfigNominalOutputReverse(0, KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.ConfigPeakOutputForward(1, KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.ConfigPeakOutputReverse(-1, KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.Config_kF(0, KaMAN_k_LinearSlidePID_Gx[E_kFF], KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.Config_kP(0, KaMAN_k_LinearSlidePID_Gx[E_kP], KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.Config_kI(0, KaMAN_k_LinearSlidePID_Gx[E_kI], KeROBO_t_MotorTimeoutMs);
  m_LinearSlide.Config_kD(0, KaMAN_k_LinearSlidePID_Gx[E_kD], KeROBO_t_MotorTimeoutMs);

  ManipulatorMotorConfigsInit(m_ArmPivotPID,
                              m_WristPID,
                              m_GripperPID,
                              m_IntakeRollersPID);

  ADAS_MN_Reset();
  ManipulatorControlInit();
#endif

  SwerveDriveMotorConfigsInit(m_frontLeftDrivePID,
                              m_frontRightDrivePID,
                              m_rearLeftDrivePID,
                              m_rearRightDrivePID);

  ADAS_Main_Init();
  ADAS_Main_Reset();

  ADAS_DM_ConfigsInit();
  ADAS_MN_ConfigsInit();
}

/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
{
  VeROBO_t_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

  Joystick1_robot_mapping(c_joyStick.GetRawButton(7),
                          c_joyStick.GetRawButton(8),
                          c_joyStick.GetRawAxis(1),
                          c_joyStick.GetRawAxis(0),
                          c_joyStick.GetRawAxis(4),
                          c_joyStick.GetRawAxis(3),
                          c_joyStick.GetRawButton(1),
                          c_joyStick.GetRawButton(3),
                          c_joyStick.GetRawButton(4),
                          c_joyStick.GetRawButton(6),
                          c_joyStick.GetRawButton(2),
                          c_joyStick.GetRawButton(5),
                          c_joyStick.GetPOV());

#ifdef CompBot
  Joystick2_robot_mapping(c_joyStick2.GetRawButton(1),
                          c_joyStick2.GetRawButton(2),
                          c_joyStick2.GetRawButton(6),
                          c_joyStick2.GetRawButton(5),
                          c_joyStick2.GetRawButton(8),
                          c_joyStick2.GetRawButton(3),
                          c_joyStick2.GetRawButton(4),
                          c_joyStick2.GetRawAxis(1),
                          c_joyStick2.GetRawAxis(4),
                          c_joyStick2.GetPOV(),
                          c_joyStick2.GetRawButton(7),
                          c_joyStick2.GetRawAxis(2),
                          c_joyStick2.GetRawAxis(3));

  Encoders_Drive_CompBot(m_encoderWheelAngleCAN_FL.GetAbsolutePosition(),
                         m_encoderWheelAngleCAN_FR.GetAbsolutePosition(),
                         m_encoderWheelAngleCAN_RL.GetAbsolutePosition(),
                         m_encoderWheelAngleCAN_RR.GetAbsolutePosition(),
                         m_encoderFrontLeftDrive,
                         m_encoderFrontRightDrive,
                         m_encoderRearLeftDrive,
                         m_encoderRearRightDrive);

  Encoders_MAN_INT(m_IntakeRollersEncoder,
                   m_ArmPivotEncoder,
                   m_GripperEncoder,
                   m_WristEncoder,
                   m_LinearSlide.GetSelectedSensorPosition(),
                   VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm],
                   m_WristreverseLimit.Get());
#else
  Encoders_Drive_PracticeBot(a_encoderFrontLeftSteer.GetVoltage(),
                             a_encoderFrontRightSteer.GetVoltage(),
                             a_encoderRearLeftSteer.GetVoltage(),
                             a_encoderRearRightSteer.GetVoltage(),
                             m_encoderFrontLeftDrive,
                             m_encoderFrontRightDrive,
                             m_encoderRearLeftDrive,
                             m_encoderRearRightDrive);
#endif

  ReadGyro2(VsCONT_s_DriverInput.b_ZeroGyro);

  VisionRun();

  DtrmnSwerveBotLocation(VeGRY_Rad_GyroYawAngleRad,
                         &VaENC_Rad_WheelAngleFwd[0],
                         &VaENC_In_WheelDeltaDistance[0],
                         VsCONT_s_DriverInput.b_ZeroGyro);

  DtrmTagOffset(V_TagID,
                VeVIS_b_TagHasTarget,
                V_TagCentered);

  ADAS_DetermineMode();

  VeADAS_e_ActiveFeature = ADAS_ControlMain(&VeADAS_Pct_SD_FwdRev,
                                            &VeADAS_Pct_SD_Strafe,
                                            &VeADAS_Pct_SD_Rotate,
                                            &VeADAS_Deg_SD_DesiredPose,
                                            &VeADAS_b_SD_RobotOriented,
                                            &VeADAS_b_X_Mode,
                                            VsCONT_s_DriverInput.b_JoystickActive,
                                            VsCONT_s_DriverInput.b_SwerveGoalAutoCenter,
                                            VeGRY_Deg_GyroYawAngleDegrees,
                                            VeODO_In_RobotDisplacementX,
                                            VeODO_In_RobotDisplacementY,
                                            VeVIS_b_TagHasTarget,
                                            VeROBO_e_RobotState,
                                            VeADAS_e_ActiveFeature,
                                            V_TagID,
                                            V_TagCentered, // comes from Vision
                                            V_TagYaw,
                                            VeROBO_e_AllianceColor,
                                            V_OffsetXOut,
                                            V_OffsetYOut,
                                            VeADAS_in_GlobalRequestX,
                                            VeADAS_in_GlobalRequestY,
                                            VeADAS_in_OffsetRequestX,
                                            VeADAS_in_OffsetRequestY);

  DriveControlMain(VsCONT_s_DriverInput.pct_SwerveForwardBack, // swerve control forward/back
                   VsCONT_s_DriverInput.pct_SwerveStrafe,      // swerve control strafe
                   VsCONT_s_DriverInput.deg_SwerveRotate,      // rotate the robot joystick
                   VsCONT_s_DriverInput.v_SwerveSpeed,         // extra speed trigger
                   VsCONT_s_DriverInput.b_SwerveRotateTo0,     // auto rotate to 0 degrees
                   VsCONT_s_DriverInput.b_SwerveRotateTo180,   // auto rotate to 180 degrees
                   VsCONT_s_DriverInput.b_ZeroGyro,
                   VeADAS_b_X_Mode, // X mode req from ADAS
                   VsCONT_s_DriverInput.b_X_Mode,
                   VeADAS_e_ActiveFeature,
                   VeADAS_Pct_SD_FwdRev,
                   VeADAS_Pct_SD_Strafe,
                   VeADAS_Pct_SD_Rotate,
                   VeADAS_Deg_SD_DesiredPose,
                   VeADAS_b_SD_RobotOriented,
                   VeGRY_Deg_GyroYawAngleDegrees,
                   VeGRY_Rad_GyroYawAngleRad,
                   VeMAN_e_CmndState,
                   VeMAN_e_AttndState,
                   &VaENC_Deg_WheelAngleFwd[0],
                   &VaENC_Deg_WheelAngleRev[0],
                   &VaDRC_RPM_WheelSpeedCmnd[0],
                   &VaDRC_Pct_WheelAngleCmnd[0]);

  LightControlMain(VeROBO_t_MatchTimeRemaining,
                   VeROBO_e_AllianceColor,
                   VsCONT_s_DriverInput.b_CameraLight,
                   VeADAS_e_ActiveFeature,
                   false,
                   false,
                   &VeLC_b_CameraLightCmndOn,
                   &VeLC_Cmd_VanityLightCmnd);

#ifdef NewVision
  frc::SmartDashboard::PutNumber("Tag Offset X", V_OffsetXOut);
  frc::SmartDashboard::PutNumber("Tag Offset Y", V_OffsetYOut);

  frc::SmartDashboard::PutNumber("Robot X", VeODO_In_RobotDisplacementX);
  frc::SmartDashboard::PutNumber("Robot Y", VeODO_In_RobotDisplacementY);
  frc::SmartDashboard::PutBoolean("Vision Button Cube", VeADAS_b_CubeAlignButtonRequest);

  VeADAS_in_GlobalRequestX = 530.0;
  VeADAS_in_GlobalRequestY = 50.0;

  // VeADAS_in_OffsetRequestX = 36.0;
  // VeADAS_in_OffsetRequestY = 18.0;

  // frc::SmartDashboard::PutNumber("GoalOffsetX", VeADAS_in_OffsetRequestX);
  // frc::SmartDashboard::PutNumber("GoalOffsetY", VeADAS_in_OffsetRequestY);

  frc::SmartDashboard::PutBoolean("has target", VeVIS_b_TagHasTarget);
  frc::SmartDashboard::PutNumber("cam1 x", V_VIS_in_TagX);
  frc::SmartDashboard::PutNumber("cam1 y", V_VIS_in_TagY);

  // frc::SmartDashboard::PutBoolean("Want to stop X", wantToStopX);
  // frc::SmartDashboard::PutBoolean("Want to stop Y", wantToStopY);

  // frc::SmartDashboard::PutNumber("cam1 z", V_Tagz);
  // frc::SmartDashboard::PutNumber("TagID ", V_TagID);
  // frc::SmartDashboard::PutNumber("TagRoll", V_TagRoll);
  // frc::SmartDashboard::PutNumber("TagPitch", V_TagPitch);
  frc::SmartDashboard::PutNumber("TagYaw", V_TagYaw);
  // frc::SmartDashboard::PutNumber("Cube Yaw", PieceCamYaw);

#endif

  /* These function calls are for test mode calibration. */
  SwerveDriveMotorConfigsCal(m_frontLeftDrivePID,
                             m_frontRightDrivePID,
                             m_rearLeftDrivePID,
                             m_rearRightDrivePID);

  ADAS_DM_ConfigsCal();
#ifdef CompBot
  ManipulatorControlMain(VeADAS_e_MAN_SchedState,
                         VeROBO_b_TestState,
                         VeADAS_b_MAN_DropObjectSlow,
                         VeADAS_b_MAN_DropObjectFast);

  ManipulatorMotorConfigsCal(m_ArmPivotPID,
                             m_WristPID,
                             m_GripperPID,
                             m_IntakeRollersPID);

  /* Set light control outputs here */
  // do_CameraLightControl.Set(VeLC_b_CameraLightCmndOn);
  m_vanityLightControler.Set(VeLC_Cmd_VanityLightCmnd);
#endif

  /* Output all of the content to the dashboard here: */
  frc::SmartDashboard::PutNumber("VeMAN_e_AttndState", (double)VeMAN_e_AttndState);
  frc::SmartDashboard::PutNumber("VeMAN_e_CmndState", (double)VeMAN_e_CmndState);
  frc::SmartDashboard::PutNumber("VeADAS_e_MAN_SchedState", (double)VeADAS_e_MAN_SchedState);

}

/******************************************************************************
 * Function:     AutonomousInit
 *S
 * Description:  Function called at init while in autonomous.  This is where we
 *               should zero out anything that we need to before autonomous mode.
 ******************************************************************************/
void Robot::AutonomousInit()
{
  VeROBO_e_RobotState = E_Auton;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();
  VeROBO_b_TestState = false;
  GyroInit();
  DriveControlInit();
  // ManipulatorControlInit();
  ADAS_Main_Reset();
  OdometryInit();
}

/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
{
  RobotMotorCommands();
}

/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  Function called when starting out in teleop mode.
 *               We should zero out all of our global varibles.
 ******************************************************************************/
void Robot::TeleopInit()
{
  VeROBO_e_RobotState = E_Teleop;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();
  VeROBO_b_TestState = false;
  V_TagCentered = false;

  ADAS_Main_Reset();
  DriveControlInit();
  // ManipulatorControlInit();
  OdometryInit();
}

/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
{

  if (VsCONT_s_DriverInput.b_CubeAlign && VeADAS_b_CubeAlignButtonPrevious == false)
  {
    VeADAS_b_CubeAlignButtonPrevious = VeADAS_b_CubeAlignButtonRequest;
    VeADAS_b_CubeAlignButtonRequest = true;
  }
  else if (VsCONT_s_DriverInput.b_CubeAlign && VeADAS_b_CubeAlignButtonPrevious == true)
  {
    VeADAS_b_CubeAlignButtonPrevious = VeADAS_b_CubeAlignButtonRequest;

    VeADAS_b_CubeAlignButtonRequest = false;
  }

  if (VsCONT_s_DriverInput.b_ConeAlign && VeADAS_b_ConeAlignButtonPrevious == false)
  {
    VeADAS_b_ConeAlignButtonPrevious = VeADAS_b_ConeAlignButtonRequest;
    VeADAS_b_ConeAlignButtonRequest = true;
  }
  else if (VsCONT_s_DriverInput.b_ConeAlign && VeADAS_b_ConeAlignButtonPrevious == true)
  {
    VeADAS_b_ConeAlignButtonPrevious = VeADAS_b_ConeAlignButtonRequest;

    VeADAS_b_ConeAlignButtonRequest = false;
  }



  RobotMotorCommands();
}

/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
{
  VeROBO_b_TestState = true;
  bool LeROBO_b_IntakeArmExtend = false;

#ifdef CompBot
  ManipulatorControlManualOverride(&VsCONT_s_DriverInput);
#endif

  if (VsCONT_s_DriverInput.b_ResetManipulatorEnocders == true)
  {
    EncodersInitCommon(m_encoderFrontRightSteer,
                       m_encoderFrontLeftSteer,
                       m_encoderRearRightSteer,
                       m_encoderRearLeftSteer,
                       m_encoderFrontRightDrive,
                       m_encoderFrontLeftDrive,
                       m_encoderRearRightDrive,
                       m_encoderRearLeftDrive);
#ifdef CompBot
    EncodersInitComp(m_ArmPivotEncoder,
                     m_WristEncoder,
                     m_GripperEncoder,
                     m_IntakeRollersEncoder);
#endif
  }

  m_frontLeftDriveMotor.Set(0);
  m_frontRightDriveMotor.Set(0);
  m_rearLeftDriveMotor.Set(0);
  m_rearRightDriveMotor.Set(0);

  m_frontLeftSteerMotor.Set(0);
  m_frontRightSteerMotor.Set(0);
  m_rearLeftSteerMotor.Set(0);
  m_rearRightSteerMotor.Set(0);

#ifdef CompBot
  m_ArmPivot.Set(VsMAN_s_Motors.k_MotorTestPower[E_MAN_ArmPivot]);
  m_Wrist.Set(VsMAN_s_Motors.k_MotorTestPower[E_MAN_Wrist]);
  m_Gripper.Set(VsMAN_s_Motors.k_MotorTestPower[E_MAN_Gripper]);
  m_IntakeRollers.Set(VsMAN_s_Motors.k_MotorTestPower[E_MAN_IntakeRollers]);
  m_LinearSlide.Set(ControlMode::PercentOutput, VsMAN_s_Motors.k_MotorTestPower[E_MAN_LinearSlide]);


  if (VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] == E_MotorExtend)
  {

    LeROBO_b_IntakeArmExtend = true;
  }
  else
  {
    LeROBO_b_IntakeArmExtend = false;
  }
  m_PCM_Valve.Set(LeROBO_b_IntakeArmExtend);
#endif
}

#ifndef RUNNING_FRC_TESTS
/******************************************************************************
 * Function:     main
 *
 * Description:  This is the main calling function for the robot.
 ******************************************************************************/
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
