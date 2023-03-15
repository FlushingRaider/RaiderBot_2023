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
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
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
  #ifdef PracticeBot
  frc::AnalogInput a_encoderFrontLeftSteer{2};
  frc::AnalogInput a_encoderFrontRightSteer{1};
  frc::AnalogInput a_encoderRearLeftSteer{3};
  frc::AnalogInput a_encoderRearRightSteer{0};
  #endif
 
  
  //DIO - Inputs / Outputs
  #ifdef CompBot
  // frc::AnalogInput      a_encoderTurret{1};
  WPI_CANCoder          m_encoderWheelAngleCAN_FL     {KeEnc_i_WheelAngleFL, "rio"};
  WPI_CANCoder          m_encoderWheelAngleCAN_FR     {KeEnc_i_WheelAngleFR, "rio"};
  WPI_CANCoder          m_encoderWheelAngleCAN_RL     {KeEnc_i_WheelAngleRL, "rio"};
  WPI_CANCoder          m_encoderWheelAngleCAN_RR     {KeEnc_i_WheelAngleRR, "rio"};
  
  // frc::DigitalInput     di_TurrentLimitSwitch {C_TurretSensorID};
  // frc::DigitalOutput    do_CameraLightControl {C_CameraLightControl_ID};
  #endif

  // PDP - Power Distribution Panel - CAN
  frc::PowerDistribution                     PDP                   {C_PDP_ID,               frc::PowerDistribution::ModuleType::kRev};

  // CAN Motor Controllers
  rev::CANSparkMax                           m_frontLeftSteerMotor {frontLeftSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontLeftDriveMotor {frontLeftDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController                 m_frontLeftDrivePID    = m_frontLeftDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_frontRightDrivePID   = m_frontRightDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_rearLeftDrivePID     = m_rearLeftDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_rearRightDrivePID    = m_rearRightDriveMotor.GetPIDController();
  
  #ifdef CompBot
  rev::CANSparkMax                           m_ArmPivot            {KeMAN_i_ArmPivot,        rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Wrist               {KeMAN_i_Wrist,           rev::CANSparkMax::MotorType::kBrushless};        
  rev::CANSparkMax                           m_Gripper             {KeMAN_i_Gripper,         rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_IntakeRollers       {KeINT_i_IntakeRollers,   rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController                 m_ArmPivotPID         = m_ArmPivot.GetPIDController();
  rev::SparkMaxPIDController                 m_WristPID            = m_Wrist.GetPIDController();
  rev::SparkMaxPIDController                 m_GripperPID          = m_Gripper.GetPIDController();
  rev::SparkMaxPIDController                 m_IntakeRollersPID    = m_IntakeRollers.GetPIDController();

  rev::SparkMaxLimitSwitch                   m_WristforwardLimit   = m_Wrist.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
  rev::SparkMaxLimitSwitch                   m_WristreverseLimit   = m_Wrist.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);

  // WPI_TalonSRX                               m_TurretRotate         {KeMAN_i_TurretRotate};
  WPI_TalonSRX                               m_LinearSlide          {KeMAN_i_LinearSlide};

  frc::Compressor                            m_pcmCompressor          {KeINT_i_PCM, frc::PneumaticsModuleType::CTREPCM};
  frc::Solenoid                              m_PCM_Valve              {KeINT_i_PCM, frc::PneumaticsModuleType::CTREPCM, 0};
  #endif

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
  rev::SparkMaxRelativeEncoder               m_ArmPivotEncoder        = m_ArmPivot.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_WristEncoder           = m_Wrist.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_GripperEncoder         = m_Gripper.GetEncoder();
  rev::SparkMaxRelativeEncoder               m_IntakeRollersEncoder   = m_IntakeRollers.GetEncoder();
  #endif

  // Driver Inputs
  frc::Joystick c_joyStick{0};
#ifdef CompBot
  frc::Joystick c_joyStick2{1};
#endif

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
#endif
