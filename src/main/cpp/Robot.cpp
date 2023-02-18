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
#include "BallHandler.hpp"
#include "LightControl.hpp"
#include "VisionV2.hpp"
#include "ADAS.hpp"
#include "ADAS_BT.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_DM.hpp"

T_RobotState V_RobotState = E_Init;
frc::DriverStation::Alliance V_AllianceColor = frc::DriverStation::Alliance::kInvalid;
double V_MatchTimeRemaining = 0;
TsRobotMotorCmnd VsRobotMotorCmnd;

/******************************************************************************
 * Function:     RobotMotorCommands
 *
 * Description:  Contains the outputs for the motors.
 ******************************************************************************/
void Robot::RobotMotorCommands()
{
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

  // Turret motor command
  double L_Temp = 0;
  if (VsCONT_s_DriverInput.e_TurretCmndDirection == E_TurrentCmndLeft)
  {
    L_Temp = -K_Pct_TurretOpenLoopCmnd;
  }
  else if (VsCONT_s_DriverInput.e_TurretCmndDirection == E_TurrentCmndRight)
  {
    L_Temp = K_Pct_TurretOpenLoopCmnd;
  }

  // m_turret.Set(ControlMode::PercentOutput, L_Temp);

#ifdef CompBot2
  // Ball launcher motors
  if (V_BH_LauncherActive == true)
  {
    m_rightShooterpid.SetReference(V_ShooterRPM_Cmnd, rev::ControlType::kVelocity);
    m_leftShooterpid.SetReference(-V_ShooterRPM_Cmnd, rev::ControlType::kVelocity);
  }
  else
  {
    // m_rightShooterMotor.Set(0); // CompBot
    m_leftShooterMotor.Set(0);
  }

  // Intake motor commands
  m_intake.Set(ControlMode::PercentOutput, V_IntakePowerCmnd); // must be positive (don't be a fool)
  m_elevator.Set(ControlMode::PercentOutput, V_ElevatorPowerCmnd);

  // XY XD lift motors
  if (VeMAN_b_ ArmInitialized == false)
  {
    m_ManTurretMotorA.Set(VeMAN_Cnt_MoterTestPowerCmndTurret);        // Turret motor
    m_ManShoulderMotorB.Set(VeMAN_Cnt_MoterTestPowerCmndArmPivot);    // Joint shoulder motor
    m_ManElevatorMotorC.Set(VeMAN_Cnt_MoterTestPowerCmndClawlevator); // Joint elevator motor
    m_ManRotateMotorD.Set(VeMAN_Cnt_MoterTestPowerCmndWrist);         // Claw rotate motor
    m_ManClawMotorE.Set(VeMAN_Cnt_MoterTestPowerCmndClaw);            // Claw open and close motor
    m_ManIntakeMotorF.Set(VeMAN_Cnt_MoterTestPowerCmndIntake);        // Intake roller motor
  }
  else
  {
    m_liftpidYD.SetReference(VeMan_Cnt_MoterCommandTurret, rev::ControlType::kPosition);   // positive is up
    m_liftpidXD.SetReference(VeMan_Cnt_MoterCommandArmPivot, rev::ControlType::kPosition); // This is temporary.  We actually want to use position, but need to force this off temporarily
  }
#endif
}

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
{
  V_RobotState = E_Init;
  V_AllianceColor = frc::DriverStation::GetAlliance();
  V_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

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
  m_Gripper.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_IntakeRollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_TurretRotate.ConfigFactoryDefault();
  m_TurretRotate.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, K_t_TurretTimeoutMs);
  m_TurretRotate.SetSensorPhase(true);
  m_TurretRotate.SetSelectedSensorPosition(0);
  m_TurretRotate.ConfigNominalOutputForward(0, K_t_TurretTimeoutMs);
  m_TurretRotate.ConfigNominalOutputReverse(0, K_t_TurretTimeoutMs);
  m_TurretRotate.ConfigPeakOutputForward(1, K_t_TurretTimeoutMs);
  m_TurretRotate.ConfigPeakOutputReverse(-1, K_t_TurretTimeoutMs);

  m_LinearSlide.ConfigFactoryDefault();
  m_LinearSlide.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, K_t_TurretTimeoutMs);
  m_LinearSlide.SetSensorPhase(true);
  m_LinearSlide.SetSelectedSensorPosition(0);
  m_LinearSlide.ConfigNominalOutputForward(0, K_t_TurretTimeoutMs);
  m_LinearSlide.ConfigNominalOutputReverse(0, K_t_TurretTimeoutMs);
  m_LinearSlide.ConfigPeakOutputForward(1, K_t_TurretTimeoutMs);
  m_LinearSlide.ConfigPeakOutputReverse(-1, K_t_TurretTimeoutMs);

  ManipulatorMotorConfigsInit(m_ArmPivotPID,
                              m_WristPID,
                              m_GripperPID,
                              m_IntakeRollersPID);
#endif

  SwerveDriveMotorConfigsInit(m_frontLeftDrivePID,
                              m_frontRightDrivePID,
                              m_rearLeftDrivePID,
                              m_rearRightDrivePID);

  ADAS_Main_Init();
  ADAS_Main_Reset();

  ADAS_DM_ConfigsInit();
  ADAS_UT_ConfigsInit();
  ADAS_BT_ConfigsInit();

  VisionInit(V_AllianceColor);
}

/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
{
  V_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

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
  // Read_IO_Sensors(di_IR_Sensor.Get(), // ball sensor upper - di_IR_Sensor.Get()
  //                 di_BallSensorLower.Get(), // ball sensor lower - di_BallSensorLower.Get()
  //                 di_XD_LimitSwitch.Get(), // XD Limit - di_XD_LimitSwitch.Get()
  //                 di_XY_LimitSwitch.Get(), // XY Limit - di_XY_LimitSwitch.Get()
  //                 false); //di_TurrentLimitSwitch.Get());

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
                   m_TurretRotate.GetSelectedSensorPosition());
#else
  Encoders_Drive_PracticeBot(a_encoderFrontLeftSteer.GetVoltage(),
                             a_encoderFrontRightSteer.GetVoltage(),
                             a_encoderRearLeftSteer.GetVoltage(),
                             a_encoderRearRightSteer.GetVoltage(),
                             m_encoderFrontLeftDrive,
                             m_encoderFrontRightDrive,
                             m_encoderRearLeftDrive,
                             m_encoderRearRightDrive);

  // Read_IO_Sensors(false, // ball sensor upper - di_IR_Sensor.Get()
  //                 false, // ball sensor lower - di_BallSensorLower.Get()
  //                 false, // XD Limit - di_XD_LimitSwitch.Get()
  //                 false, // XY Limit - di_XY_LimitSwitch.Get()
  //                 di_TurrentLimitSwitch.Get()); //di_TurrentLimitSwitch.Get());
#endif

  ReadGyro2(VsCONT_s_DriverInput.b_ZeroGyro);

  DtrmnSwerveBotLocation(VeGRY_Rad_GyroYawAngleRad,
                         &VaENC_Rad_WheelAngleFwd[0],
                         &VaENC_In_WheelDeltaDistance[0],
                         VsCONT_s_DriverInput.b_ZeroGyro);

  ADAS_DetermineMode();
#ifdef CompBot2
  V_ADAS_ActiveFeature = ADAS_ControlMain(&V_ADAS_Pct_SD_FwdRev,
                                          &V_ADAS_Pct_SD_Strafe,
                                          &V_ADAS_Pct_SD_Rotate,
                                          &V_ADAS_RPM_BH_Launcher,
                                          &V_ADAS_Pct_BH_Intake,
                                          &V_ADAS_Pct_BH_Elevator,
                                          &V_ADAS_CameraUpperLightCmndOn,
                                          &V_ADAS_CameraLowerLightCmndOn,
                                          &V_ADAS_SD_RobotOriented,
                                          &V_ADAS_Vision_RequestedTargeting,
                                          VsCONT_s_DriverInput.b_JoystickActive,
                                          VsCONT_s_DriverInput.b_SwerveGoalAutoCenter,
                                          VsCONT_s_DriverInput.b_AutoIntake,
                                          VeGRY_Deg_GyroYawAngleDegrees,
                                          VeODO_In_RobotDisplacementX,
                                          VeODO_In_RobotDisplacementY,
                                          VeVIS_b_VisionTargetAquired[E_CamTop],
                                          VeVIS_Deg_VisionYaw[E_CamTop],
                                          VeVIS_m_VisionTargetDistance[E_CamTop],
                                          VeVIS_b_VisionTargetAquired[E_CamBottom],
                                          VeVIS_Deg_VisionYaw[E_CamBottom],
                                          VeVIS_m_VisionTargetDistance[E_CamBottom],
                                          V_RobotState,
                                          VsRobotSensors.b_BallDetectedUpper,
                                          VsRobotSensors.b_BallDetectedLower,
                                          V_ADAS_ActiveFeature,
                                          V_Tagx,
                                          V_Tagy,
                                          V_TagID,
                                          V_TagYaw,
                                          V_AllianceColor);
#endif

  DriveControlMain(VsCONT_s_DriverInput.pct_SwerveForwardBack, // swerve control forward/back
                   VsCONT_s_DriverInput.pct_SwerveStrafe,      // swerve control strafe
                   VsCONT_s_DriverInput.deg_SwerveRotate,      // rotate the robot joystick
                   VsCONT_s_DriverInput.v_SwerveSpeed,         // extra speed trigger
                   VsCONT_s_DriverInput.b_SwerveRotateTo0,     // auto rotate to 0 degrees
                   VsCONT_s_DriverInput.b_ZeroGyro,            // auto rotate to 90 degrees
                   VsCONT_s_DriverInput.b_RobotFieldOrientedReq,
                   V_ADAS_ActiveFeature,
                   V_ADAS_Pct_SD_FwdRev,
                   V_ADAS_Pct_SD_Strafe,
                   V_ADAS_Pct_SD_Rotate,
                   V_ADAS_SD_RobotOriented,
                   VeGRY_Deg_GyroYawAngleDegrees,
                   VeGRY_Rad_GyroYawAngleRad,
                   &VaENC_Deg_WheelAngleFwd[0],
                   &VaENC_Deg_WheelAngleRev[0],
                   &VaDRC_RPM_WheelSpeedCmnd[0],
                   &VaDRC_Pct_WheelAngleCmnd[0]);

  // BallHandlerControlMain( VsCONT_s_DriverInput.b_IntakeIn,
  //                         VsCONT_s_DriverInput.b_IntakeOut,
  //                         VsRobotSensors.b_BallDetectedUpper,
  //                         VsRobotSensors.b_BallDetectedLower,
  //                         VsCONT_s_DriverInput.b_IntakeArmIn,
  //                         VsCONT_s_DriverInput.b_IntakeArmOut,
  //                         false,
  //                         false,
  //                         0.0,
  //                         V_ADAS_ActiveFeature,
  //                         V_ADAS_RPM_BH_Launcher,
  //                         V_ADAS_Pct_BH_Intake,
  //                         V_ADAS_Pct_BH_Elevator,
  //                        &V_IntakePowerCmnd,
  //                        &V_ElevatorPowerCmnd,
  //                        &V_ShooterRPM_Cmnd);

  LightControlMain(V_MatchTimeRemaining,
                   V_AllianceColor,
                   VsCONT_s_DriverInput.b_CameraLight,
                   V_ADAS_ActiveFeature,
                   V_ADAS_CameraUpperLightCmndOn,
                   V_ADAS_CameraLowerLightCmndOn,
                   &VeLC_b_CameraLightCmndOn,
                   &VeLC_Cmd_VanityLightCmnd);

#ifdef OldVision
  VisionRun(pc_Camera1.GetLatestResult(),
            pc_Camera2.GetLatestResult(),
            V_ADAS_Vision_RequestedTargeting,
            VsCONT_s_DriverInput.b_VisionDriverModeOverride,
            &VeVIS_b_VisionDriverRequestedModeCmnd);

  pc_Camera1.SetDriverMode(VeVIS_b_VisionDriverRequestedModeCmnd);
  pc_Camera2.SetDriverMode(VeVIS_b_VisionDriverRequestedModeCmnd);

  if (VeVIS_b_VisionDriverRequestedModeCmnd == false)
  {
    // pc_Camera1.SetPipelineIndex(VnVIS_int_VisionCameraIndex[E_Cam1]);  // Shouldn't need this one so long as Cam1 remains as top
    pc_Camera2.SetPipelineIndex(VnVIS_int_VisionCameraIndex[E_Cam2]); // Need to comment this out if Photon Vision is being calibrated/tweaked
  }
#endif

#ifdef NewVision

  VisionRun();
  frc::SmartDashboard::PutBoolean("has target", VeVIS_b_TagHasTarget);
  frc::SmartDashboard::PutNumber("cam1 x", V_Tagx);
  frc::SmartDashboard::PutNumber("cam1 y", V_Tagy);
  frc::SmartDashboard::PutNumber("cam1 z", V_Tagz);
  frc::SmartDashboard::PutNumber("TagID ", V_TagID);
  frc::SmartDashboard::PutNumber("TagRoll", V_TagRoll);
  frc::SmartDashboard::PutNumber("TagPitch", V_TagPitch);
  frc::SmartDashboard::PutNumber("TagYaw", V_TagYaw);

#endif

#ifdef CompBot2
  VeMAN_CnT_Man_DoesStuffMaybe = ManipulatorControlDictator(VsCONT_s_DriverInput.b_LiftControl,
                                                            false,
                                                            VsCONT_s_DriverInput.e_LiftCmndDirection,
                                                            V_MatchTimeRemaining,
                                                            VeMAN_CnT_Man_DoesStuffMaybe,
                                                            0,
                                                            0,
                                                            &VeMan_Cnt_MoterCommandTurret,
                                                            &VeMan_Cnt_MoterCommandArmPivot,
                                                            &VeMAN_Cnt_MoterTestPowerCmndTurret,
                                                            &VeMAN_Cnt_MoterTestPowerCmndArmPivot,
                                                            VsRobotSensors.b_XY_LimitDetected,
                                                            VsRobotSensors.b_XD_LimitDetected,
                                                            VeGRY_Deg_GyroYawAngleDegrees,
                                                            m_ManTurretMotorA.GetOutputCurrent(),
                                                            m_ManShoulderMotorB.GetOutputCurrent(),
                                                            m_ManElevatorMotorC.GetOutputCurrent(),
                                                            m_ManRotateMotorD.GetOutputCurrent(),
                                                            m_ManClawMotorE.GetOutputCurrent(),
                                                            m_ManIntakeMotorF.GetOutputCurrent(),
                                                            m_encoderLiftYD,
                                                            m_encoderLiftXD);
#endif

  /* These function calls are for test mode calibration. */
  SwerveDriveMotorConfigsCal(m_frontLeftDrivePID,
                             m_frontRightDrivePID,
                             m_rearLeftDrivePID,
                             m_rearRightDrivePID);
#ifdef CompBot2
  BallHandlerMotorConfigsCal(m_rightShooterpid,
                             m_leftShooterpid);

  ManipulatorMotorConfigsCal(m_liftpidYD,
                      m_liftpidXD);
#endif
  ADAS_UT_ConfigsCal();

  ADAS_BT_ConfigsCal();

  /* Output all of the content to the dashboard here: */
  frc::SmartDashboard::PutNumber("RobotDisplacementY", VeODO_In_RobotDisplacementY);
  frc::SmartDashboard::PutNumber("RobotDisplacementX", VeODO_In_RobotDisplacementX);

  /* Set light control outputs here */
  do_CameraLightControl.Set(VeLC_b_CameraLightCmndOn);
#ifdef CompBot
  m_vanityLightControler.Set(VeLC_Cmd_VanityLightCmnd);
#endif
}

/******************************************************************************
 * Function:     AutonomousInit
 *S
 * Description:  Function called at init while in autonomous.  This is where we
 *               should zero out anything that we need to before autonomous mode.
 ******************************************************************************/
void Robot::AutonomousInit()
{
  V_RobotState = E_Auton;
  V_AllianceColor = frc::DriverStation::GetAlliance();
  GyroInit();
  DriveControlInit();
    BallHandlerInit();
    ManipulatorControlInit();
  ADAS_Main_Reset();
  OdometryInit();

  VisionInit(V_AllianceColor);
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
  V_RobotState = E_Teleop;
  V_AllianceColor = frc::DriverStation::GetAlliance();

  ADAS_Main_Reset();
  DriveControlInit();
  BallHandlerInit();
  ManipulatorControlInit();
  OdometryInit();
  VisionInit(V_AllianceColor);
#ifdef CompBot2
  m_encoderrightShooter.SetPosition(0);
  m_encoderleftShooter.SetPosition(0);
#endif
  // m_turret.SetSelectedSensorPosition(0);
}

/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
{
  RobotMotorCommands();
}

/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
{
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

  m_TurretRotate.Set(ControlMode::PercentOutput, VsMAN_s_Motors.k_MotorTestPower[E_MAN_Turret]);
  m_LinearSlide.Set(ControlMode::PercentOutput, VsMAN_s_Motors.k_MotorTestPower[E_MAN_LinearSlide]);
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
