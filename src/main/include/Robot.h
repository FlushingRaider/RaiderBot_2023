#ifndef ROBOT
#define ROBOT

#pragma once

#include <string>

#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>
#include <frc/DutyCycleEncoder.h>
// #include <networktables/NetworkTable.h>
#include <photonlib/PhotonCamera.h>
// #include <photonlib/PhotonUtils.h>
// #include <cstdio>
// #include <cameraserver/CameraServer.h>

#include "Const.hpp"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void RobotMotorCommands();

  // Analog Inputs Test
  // Practice Bot Wheel Angle Encoders
  frc::AnalogInput a_encoderFrontLeftSteer{2};
  frc::AnalogInput a_encoderFrontRightSteer{1};
  frc::AnalogInput a_encoderRearLeftSteer{3};
  frc::AnalogInput a_encoderRearRightSteer{0};
 
  
  //DIO - Inputs / Outputs
  #ifdef CompBot
  frc::DutyCycleEncoder a_encoderWheelAngleFrontLeft  {C_MagEncoderFL_ID};
  frc::DutyCycleEncoder a_encoderWheelAngleFrontRight {C_MagEncoderFR_ID};
  frc::DutyCycleEncoder a_encoderWheelAngleRearLeft   {C_MagEncoderRL_ID};
  frc::DutyCycleEncoder a_encoderWheelAngleRearRight  {C_MagEncoderRR_ID};

  frc::DigitalInput     di_XY_LimitSwitch     {C_XY_LimitSwitch_ID};
  frc::DigitalInput     di_XD_LimitSwitch     {C_XD_LimitSwitch_ID};

  frc::DigitalInput     di_IR_Sensor          {C_IR_Sensor_ID};
  frc::DigitalInput     di_BallSensorLower    {C_LowerBallSensorID};
  #endif

  frc::DigitalInput     di_TurrentLimitSwitch {C_TurretSensorID};

  frc::DigitalOutput    do_CameraLightControl {C_CameraLightControl_ID};

  // PDP - Power Distribution Panel - CAN
  frc::PowerDistribution                     PDP                   {C_PDP_ID,               frc::PowerDistribution::ModuleType::kCTRE};

  // CAN Motor Controllers
  rev::CANSparkMax                           m_frontLeftSteerMotor {frontLeftSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontLeftDriveMotor {frontLeftDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
#ifdef CompBot
  rev::CANSparkMax                           m_rightShooterMotor   {rightShooterID,          rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_leftShooterMotor    {leftShooterID,           rev::CANSparkMax::MotorType::kBrushless};
                        
  rev::CANSparkMax                           m_liftMotorYD         {C_liftYD_ID,             rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_liftMotorXD         {C_liftXD_ID,             rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController                 m_rightShooterpid        = m_rightShooterMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_leftShooterpid         = m_leftShooterMotor.GetPIDController();
       
  rev::SparkMaxPIDController                 m_liftpidYD              = m_liftMotorYD.GetPIDController();
  rev::SparkMaxPIDController                 m_liftpidXD              = m_liftMotorXD.GetPIDController();

  ctre::phoenix::motorcontrol::can::TalonSRX m_intake              {C_intakeID};
  ctre::phoenix::motorcontrol::can::TalonSRX m_elevator            {C_elevatorID};
#endif
  rev::SparkMaxPIDController                 m_frontLeftDrivePID      = m_frontLeftDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_frontRightDrivePID     = m_frontRightDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_rearLeftDrivePID       = m_rearLeftDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_rearRightDrivePID      = m_rearRightDriveMotor.GetPIDController();

  // PWM Motor / Light Controllers
  #ifdef CompBot
  frc::Spark                                 m_vanityLightControler {C_VanityLight_ID};
  #endif

  // CAN Encoders
  rev::SparkMaxRelativeEncoder               m_encoderFrontLeftSteer  = m_frontLeftSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderFrontLeftDrive  = m_frontLeftDriveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderFrontRightSteer = m_frontRightSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderFrontRightDrive = m_frontRightDriveMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder               m_encoderRearLeftSteer   = m_rearLeftSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderRearLeftDrive   = m_rearLeftDriveMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderRearRightSteer  = m_rearRightSteerMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderRearRightDrive  = m_rearRightDriveMotor.GetEncoder();
#ifdef CompBot
  rev::SparkMaxRelativeEncoder               m_encoderrightShooter    = m_rightShooterMotor.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderleftShooter     = m_leftShooterMotor.GetEncoder();

  rev::SparkMaxRelativeEncoder               m_encoderLiftYD          = m_liftMotorYD.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_encoderLiftXD          = m_liftMotorXD.GetEncoder();
#endif

  WPI_TalonSRX                               m_turret{C_turretID};

  // Driver Inputs
  frc::Joystick c_joyStick{0};
#ifdef CompBot
  // frc::Joystick c_joyStick2{1};
#endif


  // Network tables
  photonlib::PhotonCamera pc_Camera1{"Top"};
  photonlib::PhotonCamera pc_Camera2{"Bottom"};
  
 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  
};

#endif