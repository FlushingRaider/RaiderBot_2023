/*
 * Team 5561 2023 Code
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
#include "Lift.hpp"
#include "BallHandler.hpp"
#include "LightControl.hpp"
#include "VisionV2.hpp"
#include "ADAS.hpp"
#include "ADAS_BT.hpp"
#include "ADAS_UT.hpp"
#include "ADAS_DM.hpp"
 
T_RobotState                 V_RobotState        = E_Init;
frc::DriverStation::Alliance V_AllianceColor     = frc::DriverStation::Alliance::kInvalid;
double                       V_MatchTimeRemaining = 0;
TsRobotMotorCmnd             VsRobotMotorCmnd;


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
  if (Ve_b_SD_DriveWheelsInPID == true)
    {
    m_frontLeftDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_FrontLeft],   rev::ControlType::kVelocity);
    m_frontRightDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_FrontRight], rev::ControlType::kVelocity);
    m_rearLeftDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_RearLeft],     rev::ControlType::kVelocity);
    m_rearRightDrivePID.SetReference(V_SD_WheelSpeedCmnd[E_RearRight],   rev::ControlType::kVelocity);
    m_frontLeftSteerMotor.Set(V_k_SD_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(V_k_SD_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(V_k_SD_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(V_k_SD_WheelAngleCmnd[E_RearRight]);
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
  if (VsDriverInput.e_TurretCmndDirection == E_TurrentCmndLeft)
    {
    L_Temp = -K_Pct_TurretOpenLoopCmnd;
    }
  else if (VsDriverInput.e_TurretCmndDirection == E_TurrentCmndRight)
    {
    L_Temp = K_Pct_TurretOpenLoopCmnd;
    }

  m_turret.Set(ControlMode::PercentOutput, L_Temp);


#ifdef CompBot
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
  m_intake.Set(ControlMode::PercentOutput, V_IntakePowerCmnd); //must be positive (don't be a fool)
  m_elevator.Set(ControlMode::PercentOutput, V_ElevatorPowerCmnd);

  // XY XD lift motors
  if (V_LiftInitialized == false)
    {
    m_liftMotorYD.Set(V_LiftYD_TestPowerCmnd);
    m_liftMotorXD.Set(V_LiftXD_TestPowerCmnd);
    }
  else
    {
    m_liftpidYD.SetReference(V_lift_command_YD, rev::ControlType::kPosition); // positive is up
    m_liftpidXD.SetReference(V_lift_command_XD, rev::ControlType::kPosition); // This is temporary.  We actually want to use position, but need to force this off temporarily
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
  V_RobotState         = E_Init;
  V_AllianceColor      = frc::DriverStation::GetAlliance();
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
  EncodersInitComp(m_encoderLiftYD,
                   m_encoderLiftXD,
                   m_encoderrightShooter,
                   m_encoderleftShooter);
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

  m_turret.ConfigFactoryDefault();
  m_turret.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, K_t_TurretTimeoutMs);
  m_turret.SetSensorPhase(true);
  m_turret.SetSelectedSensorPosition(0);
  m_turret.ConfigNominalOutputForward(0, K_t_TurretTimeoutMs);
  m_turret.ConfigNominalOutputReverse(0, K_t_TurretTimeoutMs);
  m_turret.ConfigPeakOutputForward(1, K_t_TurretTimeoutMs);
  m_turret.ConfigPeakOutputReverse(-1, K_t_TurretTimeoutMs);

  #ifdef CompBot
  m_liftMotorYD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_liftMotorXD.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_rightShooterMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_leftShooterMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  #endif



  SwerveDriveMotorConfigsInit(m_frontLeftDrivePID,
                              m_frontRightDrivePID,
                              m_rearLeftDrivePID,
                              m_rearRightDrivePID);

  #ifdef CompBot
  BallHandlerMotorConfigsInit(m_rightShooterpid,
                              m_leftShooterpid);

  LiftMotorConfigsInit(m_liftpidYD,
                       m_liftpidXD);
  #endif

  ADAS_Main_Init();
  ADAS_Main_Reset();

  ADAS_DM_ConfigsInit();
  ADAS_UT_ConfigsInit();
  ADAS_BT_ConfigsInit();

  VisionRobotInit();

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

#ifdef CompBot
  Read_IO_Sensors(di_IR_Sensor.Get(), // ball sensor upper - di_IR_Sensor.Get()
                  di_BallSensorLower.Get(), // ball sensor lower - di_BallSensorLower.Get()
                  di_XD_LimitSwitch.Get(), // XD Limit - di_XD_LimitSwitch.Get()
                  di_XY_LimitSwitch.Get(), // XY Limit - di_XY_LimitSwitch.Get()
                  false); //di_TurrentLimitSwitch.Get());
  
  Joystick1_robot_mapping(c_joyStick2.GetRawButton(1),
                          c_joyStick2.GetRawButton(2),
                          c_joyStick2.GetRawButton(6), 
                          c_joyStick2.GetRawButton(5),
                          c_joyStick2.GetRawButton(8),
                          c_joyStick2.GetRawButton(3),
                          c_joyStick2.GetRawButton(4),
                          c_joyStick2.GetRawAxis(1),
                          c_joyStick2.GetRawAxis(5),
                          c_joyStick2.GetPOV(),
                          c_joyStick2.GetRawButton(7));

  Read_Encoders(a_encoderWheelAngleFrontLeft.Get().value(),
                a_encoderWheelAngleFrontRight.Get().value(),
                a_encoderWheelAngleRearLeft.Get().value(),
                a_encoderWheelAngleRearRight.Get().value(),
                a_encoderFrontLeftSteer.GetVoltage(),
                a_encoderFrontRightSteer.GetVoltage(),
                a_encoderRearLeftSteer.GetVoltage(),
                a_encoderRearRightSteer.GetVoltage(),
                m_encoderFrontLeftDrive,
                m_encoderFrontRightDrive,
                m_encoderRearLeftDrive,
                m_encoderRearRightDrive,
                m_encoderrightShooter,
                m_encoderleftShooter,
                m_encoderLiftYD,
                m_encoderLiftXD,
                m_turret.GetSelectedSensorPosition(1));  //m_turret.GetSelectedSensorPosition()
#else
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

  Read_Encoders2(a_encoderFrontLeftSteer.GetVoltage(),
                 a_encoderFrontRightSteer.GetVoltage(),
                 a_encoderRearLeftSteer.GetVoltage(),
                 a_encoderRearRightSteer.GetVoltage(),
                 m_encoderFrontLeftDrive,
                 m_encoderFrontRightDrive,
                 m_encoderRearLeftDrive,
                 m_encoderRearRightDrive,
                 m_turret.GetSelectedSensorPosition(1)); 

  Read_IO_Sensors(false, // ball sensor upper - di_IR_Sensor.Get()
                  false, // ball sensor lower - di_BallSensorLower.Get()
                  false, // XD Limit - di_XD_LimitSwitch.Get()
                  false, // XY Limit - di_XY_LimitSwitch.Get()
                  di_TurrentLimitSwitch.Get()); //di_TurrentLimitSwitch.Get());
#endif

  ReadGyro2(VsDriverInput.b_ZeroGyro);

  DtrmnSwerveBotLocation( V_GyroYawAngleRad,
                         &V_Rad_WheelAngleFwd[0],
                         &V_M_WheelDeltaDistance[0]);

  ADAS_DetermineMode();

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
                                           VsDriverInput.b_JoystickActive,
                                           VsDriverInput.b_StopShooterAutoClimbResetGyro,
                                           VsDriverInput.b_SwerveGoalAutoCenter,
                                           VsDriverInput.b_AutoIntake,
                                           V_GyroYawAngleDegrees,
                                           V_l_RobotDisplacementX,
                                           V_l_RobotDisplacementY,
                                           V_VisionTargetAquired[E_CamTop],
                                           V_VisionYaw[E_CamTop],
                                           V_VisionTargetDistanceMeters[E_CamTop],
                                           V_VisionTargetAquired[E_CamBottom],
                                           V_VisionYaw[E_CamBottom],
                                           V_VisionTargetDistanceMeters[E_CamBottom],
                                           V_RobotState,
                                           V_ShooterSpeedCurr,
                                           VsRobotSensors.b_BallDetectedUpper,
                                           VsRobotSensors.b_BallDetectedLower,
                                           VsDriverInput.b_ElevatorUp,
                                           VsDriverInput.b_ElevatorDown,
                                           VsDriverInput.b_IntakeIn,
                                           V_ADAS_ActiveFeature);

  DriveControlMain( VsDriverInput.pct_SwerveForwardBack,  // swerve control forward/back
                    VsDriverInput.pct_SwerveStrafe,  // swerve control strafe
                    VsDriverInput.deg_SwerveRotate,  // rotate the robot joystick
                    VsDriverInput.v_SwerveSpeed,   // extra speed trigger
                    VsDriverInput.b_SwerveRotateTo0, // auto rotate to 0 degrees
                    VsDriverInput.b_SwerveRotateTo90, // auto rotate to 90 degrees
                    VsDriverInput.b_RobotFieldOrientedReq,
                    V_ADAS_ActiveFeature,
                    V_ADAS_Pct_SD_FwdRev,
                    V_ADAS_Pct_SD_Strafe,
                    V_ADAS_Pct_SD_Rotate,
                    V_ADAS_SD_RobotOriented,
                    V_GyroYawAngleDegrees,
                    V_GyroYawAngleRad,
                   &V_Deg_WheelAngleFwd[0],
                   &V_Deg_WheelAngleRev[0],
                   &V_SD_WheelSpeedCmnd[0],
                   &V_k_SD_WheelAngleCmnd[0]);

  BallHandlerControlMain( VsDriverInput.b_IntakeIn,
                          VsDriverInput.b_IntakeOut,
                          VsRobotSensors.b_BallDetectedUpper,
                          VsRobotSensors.b_BallDetectedLower,
                          VsDriverInput.b_ElevatorUp,
                          VsDriverInput.b_ElevatorDown,
                          VsDriverInput.b_StopShooterAutoClimbResetGyro,
                          VsDriverInput.b_AutoSetSpeedShooter,
                          V_ShooterSpeedCurr,
                          VsDriverInput.pct_ManualShooterDesiredSpeed,
                          V_ADAS_ActiveFeature,
                          V_ADAS_RPM_BH_Launcher,
                          V_ADAS_Pct_BH_Intake,
                          V_ADAS_Pct_BH_Elevator,
                         &V_IntakePowerCmnd,
                         &V_ElevatorPowerCmnd,
                         &V_ShooterRPM_Cmnd);

  LightControlMain(V_MatchTimeRemaining,
                   V_AllianceColor,
                   VsDriverInput.b_CameraLight,
                   V_ADAS_ActiveFeature,
                   V_ADAS_CameraUpperLightCmndOn,
                   V_ADAS_CameraLowerLightCmndOn,
                  &V_CameraLightCmndOn,
                  &V_VanityLightCmnd);

  VisionRun( pc_Camera1.GetLatestResult(),
             pc_Camera2.GetLatestResult(),
             V_ADAS_Vision_RequestedTargeting,
             VsDriverInput.b_VisionDriverModeOverride,
            &V_VisionDriverModeCmndFinal);

  pc_Camera1.SetDriverMode(V_VisionDriverModeCmndFinal);
  pc_Camera2.SetDriverMode(V_VisionDriverModeCmndFinal);

  if (V_VisionDriverModeCmndFinal == false)
    {
    // pc_Camera1.SetPipelineIndex(V_VisionCameraIndex[E_Cam1]);  // Shouldn't need this one so long as Cam1 remains as top
    pc_Camera2.SetPipelineIndex(V_VisionCameraIndex[E_Cam2]);  // Need to comment this out if Photon Vision is being calibrated/tweaked
    }

  #ifdef CompBot
  V_Lift_state = Lift_Control_Dictator(VsDriverInput.b_LiftControl,
                                       VsDriverInput.b_StopShooterAutoClimbResetGyro,
                                       VsDriverInput.e_LiftCmndDirection,
                                       V_MatchTimeRemaining,
                                       V_Lift_state,
                                       V_LiftPostitionYD,
                                       V_LiftPostitionXD,
                                       &V_lift_command_YD,
                                       &V_lift_command_XD,
                                       &V_LiftYD_TestPowerCmnd,
                                       &V_LiftXD_TestPowerCmnd,
                                       VsRobotSensors.b_XY_LimitDetected,
                                       VsRobotSensors.b_XD_LimitDetected,
                                       V_GyroYawAngleDegrees,
                                       m_liftMotorYD.GetOutputCurrent(),
                                       m_liftMotorXD.GetOutputCurrent(),
                                       m_encoderLiftYD,
                                       m_encoderLiftXD);
  #endif

  /* These function calls are for test mode calibration. */
  SwerveDriveMotorConfigsCal(m_frontLeftDrivePID,
                             m_frontRightDrivePID,
                             m_rearLeftDrivePID,
                             m_rearRightDrivePID);
  #ifdef CompBot
  BallHandlerMotorConfigsCal(m_rightShooterpid,
                             m_leftShooterpid);

  LiftMotorConfigsCal(m_liftpidYD,
                      m_liftpidXD);
  #endif
  ADAS_UT_ConfigsCal();

  ADAS_BT_ConfigsCal();

/* Output all of the content to the dashboard here: */
  frc::SmartDashboard::PutBoolean("Turret",         VsRobotSensors.b_TurretZero);
  frc::SmartDashboard::PutNumber("TurretPosition",  V_TurretPosition);

  /* Set light control outputs here */
  do_CameraLightControl.Set(V_CameraLightCmndOn);
  #ifdef CompBot
  m_vanityLightControler.Set(V_VanityLightCmnd);
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
    LiftControlInit();
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
  LiftControlInit();
  OdometryInit();
  VisionInit(V_AllianceColor);
  #ifdef CompBot
  m_encoderrightShooter.SetPosition(0);
  m_encoderleftShooter.SetPosition(0);
  #endif
  m_turret.SetSelectedSensorPosition(0);
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
  Lift_Control_ManualOverride(&V_LiftYD_TestPowerCmnd,
                              &V_LiftXD_TestPowerCmnd,
                               m_liftMotorYD.GetOutputCurrent(),
                               m_liftMotorXD.GetOutputCurrent(),
                               VsDriverInput.e_LiftCmndDirection,
                               VsRobotSensors.b_XY_LimitDetected,
                               VsRobotSensors.b_XD_LimitDetected);
  #endif
 
  if (VsDriverInput.b_StopShooterAutoClimbResetGyro == true)
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
    EncodersInitComp(m_encoderLiftYD,
                     m_encoderLiftXD,
                     m_encoderrightShooter,
                     m_encoderleftShooter);
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
  m_liftMotorYD.Set(V_LiftYD_TestPowerCmnd);
  m_liftMotorXD.Set(V_LiftXD_TestPowerCmnd);

  m_rightShooterMotor.Set(0);
  m_leftShooterMotor.Set(0);

  m_intake.Set(ControlMode::PercentOutput, 0);
  m_elevator.Set(ControlMode::PercentOutput, 0);
  #endif
  
  m_turret.SetSelectedSensorPosition(0);
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
