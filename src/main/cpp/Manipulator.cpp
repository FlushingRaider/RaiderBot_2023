/*
  Manipulator.cpp

   Created on: Feb 01, 2022
   Author: Lauren and Chloe

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift go brrrrrrrrrrrrrrrrrrr -chloe
 */

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"
#include "ADAS_MN.hpp"

TeMAN_ManipulatorStates VeMAN_e_CmndState  = E_MAN_Init; // What is our next/current step?
TeMAN_ManipulatorStates VeMAN_e_AttndState = E_MAN_Init; // What is our desired end state?

TeMAN_MotorControl VsMAN_s_Motors; // All of the motor commands for the manipulator/intake motors
TeMAN_MotorControl VsMAN_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeMAN_MotorControl VsMAN_s_MotorsTest; // Temporary commands for the motors, not the final output
TsMAN_Sensor       VsMAN_s_Sensors; // All of the sensor values for the manipulator/intake motors
double             VaMAN_k_PositionToEncoder[E_MAN_Sz]; // Conversion value to go from desired manipulator position to equivalent encoder position

double             VaMAN_Deg_TurretAngleError;
double             VaMAN_k_TurretAngleIntegral;
double             VaMAN_In_LinearSlideError;
double             VaMAN_k_LinearSlideIntegral;

double VaMAN_k_ArmPivotPID_Gx[E_PID_SparkMaxCalSz];
double VaMAN_k_WristPID_Gx[E_PID_SparkMaxCalSz];
double VaMAN_k_GripperPID_Gx[E_PID_SparkMaxCalSz];
double VaMAN_k_IntakeRollersPID_Gx[E_PID_SparkMaxCalSz];
double VaMAN_k_TurretPID_Gx[E_PID_CalSz];
double VaMAN_k_LinearSlidePID_Gx[E_PID_CalSz];

bool   VeMAN_b_CriteriaMet = false;
bool   VeMAN_b_Paused = false; //Checks to see if paused (for testing)

#ifdef Manipulator_Test
bool   VeMAN_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool VeMAN_b_TestState = false;
#endif

/******************************************************************************
 * Function:     ManipulatorMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the Arm and intake motors.
 ******************************************************************************/
void ManipulatorMotorConfigsInit(rev::SparkMaxPIDController m_ArmPivotPID,
                                 rev::SparkMaxPIDController m_WristPID,
                                 rev::SparkMaxPIDController m_GripperPID,
                                 rev::SparkMaxPIDController m_IntakeRollersPID)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;

  // set PID coefficients
  m_ArmPivotPID.SetP(KaMAN_k_ArmPivotPID_Gx[E_kP]);
  m_ArmPivotPID.SetI(KaMAN_k_ArmPivotPID_Gx[E_kI]);
  m_ArmPivotPID.SetD(KaMAN_k_ArmPivotPID_Gx[E_kD]);
  m_ArmPivotPID.SetIZone(KaMAN_k_ArmPivotPID_Gx[E_kIz]);
  m_ArmPivotPID.SetFF(KaMAN_k_ArmPivotPID_Gx[E_kFF]);
  m_ArmPivotPID.SetOutputRange(KaMAN_k_ArmPivotPID_Gx[E_kMinOutput], KaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]);

  m_WristPID.SetP(KaMAN_k_WristPID_Gx[E_kP]);
  m_WristPID.SetI(KaMAN_k_WristPID_Gx[E_kI]);
  m_WristPID.SetD(KaMAN_k_WristPID_Gx[E_kD]);
  m_WristPID.SetIZone(KaMAN_k_WristPID_Gx[E_kIz]);
  m_WristPID.SetFF(KaMAN_k_WristPID_Gx[E_kFF]);
  m_WristPID.SetOutputRange(KaMAN_k_WristPID_Gx[E_kMinOutput], KaMAN_k_WristPID_Gx[E_kMaxOutput]);

  m_GripperPID.SetP(KaMAN_k_GripperPID_Gx[E_kP]);
  m_GripperPID.SetI(KaMAN_k_GripperPID_Gx[E_kI]);
  m_GripperPID.SetD(KaMAN_k_GripperPID_Gx[E_kD]);
  m_GripperPID.SetIZone(KaMAN_k_GripperPID_Gx[E_kIz]);
  m_GripperPID.SetFF(KaMAN_k_GripperPID_Gx[E_kFF]);
  m_GripperPID.SetOutputRange(KaMAN_k_GripperPID_Gx[E_kMinOutput], KaMAN_k_GripperPID_Gx[E_kMaxOutput]);

  m_IntakeRollersPID.SetP(KaMAN_k_IntakeRollersPID_Gx[E_kP]);
  m_IntakeRollersPID.SetI(KaMAN_k_IntakeRollersPID_Gx[E_kI]);
  m_IntakeRollersPID.SetD(KaMAN_k_IntakeRollersPID_Gx[E_kD]);
  m_IntakeRollersPID.SetIZone(KaMAN_k_IntakeRollersPID_Gx[E_kIz]);
  m_IntakeRollersPID.SetFF(KaMAN_k_IntakeRollersPID_Gx[E_kFF]);
  m_IntakeRollersPID.SetOutputRange(KaMAN_k_IntakeRollersPID_Gx[E_kMinOutput], KaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]);

  for (LeMAN_i_Index = E_MAN_Turret;
       LeMAN_i_Index < E_MAN_Sz;
       LeMAN_i_Index = TeMAN_e_ManipulatorActuator(int(LeMAN_i_Index) + 1))
    {
      VsMAN_s_Motors.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTemp.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTest.k_MotorCmnd[LeMAN_i_Index] = 0.0;
    }

  VaMAN_k_PositionToEncoder[E_MAN_Turret] = KeENC_k_TurretEncoderScaler;
  VaMAN_k_PositionToEncoder[E_MAN_ArmPivot] = KeENC_k_ArmPivot;
  VaMAN_k_PositionToEncoder[E_MAN_LinearSlide] = KeENC_k_LinearSlideEncoderScaler;
  VaMAN_k_PositionToEncoder[E_MAN_Wrist] = KeENC_Deg_Wrist;
  VaMAN_k_PositionToEncoder[E_MAN_Gripper] = KeENC_RPM_Gripper;
  VaMAN_k_PositionToEncoder[E_MAN_IntakeRollers] = KeENC_RPM_IntakeRollers;
  VaMAN_k_PositionToEncoder[E_MAN_IntakeArm] = 1.0;

  VsMAN_s_Motors.k_MotorRampRate[E_MAN_Turret] = KeMAN_DegS_TurretRate;
  VsMAN_s_Motors.k_MotorRampRate[E_MAN_ArmPivot] = KeMAN_DegS_ArmPivotRate;
  VsMAN_s_Motors.k_MotorRampRate[E_MAN_LinearSlide] = KeMAN_InS_LinearSlideRate;
  VsMAN_s_Motors.k_MotorRampRate[E_MAN_Wrist] = KeMAN_DegS_WristRate;
  VsMAN_s_Motors.k_MotorRampRate[E_MAN_Gripper] = KeMAN_DegS_GripperRate;
  VsMAN_s_Motors.k_MotorRampRate[E_MAN_IntakeRollers] = KeMAN_RPMS_IntakeRate;

  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Turret] = KeMAN_DegS_TurretRate;
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot] = KeMAN_DegS_ArmPivotRate;
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide] = KeMAN_InS_LinearSlideRate;
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist] = KeMAN_DegS_WristRate;
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Gripper] = KeMAN_DegS_GripperRate;
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_IntakeRollers] = KeMAN_RPMS_IntakeRate;

  VaMAN_Deg_TurretAngleError = 0.0;
  VaMAN_k_TurretAngleIntegral = 0.0;
  VaMAN_In_LinearSlideError = 0.0;
  VaMAN_k_LinearSlideIntegral = 0.0;

  #ifdef Manipulator_Test
  T_PID_SparkMaxCal LeMAN_i_Index2 = E_kP;
  T_PID_Cal LeMAN_i_Index3 = E_P_Gx;

  for (LeMAN_i_Index2 = E_kP;
       LeMAN_i_Index2 < E_PID_SparkMaxCalSz;
       LeMAN_i_Index2 = T_PID_SparkMaxCal(int(LeMAN_i_Index2) + 1))
    {
    VaMAN_k_ArmPivotPID_Gx[LeMAN_i_Index2] = KaMAN_k_ArmPivotPID_Gx[LeMAN_i_Index2];
    VaMAN_k_WristPID_Gx[LeMAN_i_Index2] = KaMAN_k_WristPID_Gx[LeMAN_i_Index2];
    VaMAN_k_GripperPID_Gx[LeMAN_i_Index2] = KaMAN_k_GripperPID_Gx[LeMAN_i_Index2];
    VaMAN_k_IntakeRollersPID_Gx[LeMAN_i_Index2] = KaMAN_k_IntakeRollersPID_Gx[LeMAN_i_Index2];
    }

  for (LeMAN_i_Index3 = E_P_Gx;
       LeMAN_i_Index3 < E_PID_CalSz;
       LeMAN_i_Index3 = T_PID_Cal(int(LeMAN_i_Index3) + 1))
    {
    VaMAN_k_TurretPID_Gx[LeMAN_i_Index3] = KaMAN_k_TurretPID_Gx[LeMAN_i_Index3];
    VaMAN_k_LinearSlidePID_Gx[LeMAN_i_Index3] = KaMAN_k_LinearSlidePID_Gx[LeMAN_i_Index3];
    }

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Wrist", KaMAN_k_WristPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Wrist", KaMAN_k_WristPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Wrist", KaMAN_k_WristPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Wrist", KaMAN_k_WristPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Wrist", KaMAN_k_WristPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Wrist", KaMAN_k_WristPID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Gripper", KaMAN_k_GripperPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Turret", KaMAN_k_TurretPID_Gx[E_P_Gx]);
  frc::SmartDashboard::PutNumber("I Gain - Turret", KaMAN_k_TurretPID_Gx[E_I_Gx]);
  frc::SmartDashboard::PutNumber("D Gain - Turret", KaMAN_k_TurretPID_Gx[E_D_Gx]);
  frc::SmartDashboard::PutNumber("I Upper - Turret", KaMAN_k_TurretPID_Gx[E_I_Ul]);
  frc::SmartDashboard::PutNumber("I Lower - Turret", KaMAN_k_TurretPID_Gx[E_I_Ll]);
  frc::SmartDashboard::PutNumber("Max Output - Turret", KaMAN_k_TurretPID_Gx[E_Max_Ul]);
  frc::SmartDashboard::PutNumber("Min Output - Turret", KaMAN_k_TurretPID_Gx[E_Max_Ll]);

  frc::SmartDashboard::PutNumber("P Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_P_Gx]);
  frc::SmartDashboard::PutNumber("I Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Gx]);
  frc::SmartDashboard::PutNumber("D Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_D_Gx]);
  frc::SmartDashboard::PutNumber("I Upper - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ul]);
  frc::SmartDashboard::PutNumber("I Lower - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ll]);
  frc::SmartDashboard::PutNumber("Max Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ul]);
  frc::SmartDashboard::PutNumber("Min Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KeMAN_DegS_TurretRate", KeMAN_DegS_TurretRate);
  frc::SmartDashboard::PutNumber("KeMAN_DegS_ArmPivotRate", KeMAN_DegS_ArmPivotRate);
  frc::SmartDashboard::PutNumber("KeMAN_InS_LinearSlideRate", KeMAN_InS_LinearSlideRate);
  frc::SmartDashboard::PutNumber("KeMAN_DegS_WristRate", KeMAN_DegS_WristRate);
  frc::SmartDashboard::PutNumber("KeMAN_DegS_GripperRate", KeMAN_DegS_GripperRate);
  frc::SmartDashboard::PutNumber("KeMAN_RPMS_IntakeRate", KeMAN_RPMS_IntakeRate);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Set Position Gripper", 0);
  frc::SmartDashboard::PutNumber("Set Position Wrist", 0);
  frc::SmartDashboard::PutNumber("Set Position Linear Slide", 0);
  frc::SmartDashboard::PutNumber("Set Position Arm Pivot", 0);
  frc::SmartDashboard::PutNumber("Set Position Turret", 0);
  frc::SmartDashboard::PutNumber("Set Speed Intake Rollers ", 0);
  frc::SmartDashboard::PutBoolean("Set Position Intake", false);
  #endif
  }


/******************************************************************************
 * Function:     ManipulatorMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the manipulator motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_ArmPivotPID,
                                rev::SparkMaxPIDController m_WristPID,
                                rev::SparkMaxPIDController m_GripperPID,
                                rev::SparkMaxPIDController m_IntakeRollersPID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef Manipulator_Test
  bool LeMAN_b_IntakePosition = false;  // false is retracted, true extended
  double L_p_Pivot   = frc::SmartDashboard::GetNumber("P Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kP]);
  double L_i_Pivot   = frc::SmartDashboard::GetNumber("I Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kI]);
  double L_d_Pivot   = frc::SmartDashboard::GetNumber("D Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kD]);
  double L_iz_Pivot  = frc::SmartDashboard::GetNumber("I Zone - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kIz]);
  double L_max_Pivot = frc::SmartDashboard::GetNumber("Max Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]);
  double L_min_Pivot = frc::SmartDashboard::GetNumber("Min Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMinOutput]);

  double L_p_Wrist   = frc::SmartDashboard::GetNumber("P Gain - Wrist", KaMAN_k_WristPID_Gx[E_kP]);
  double L_i_Wrist   = frc::SmartDashboard::GetNumber("I Gain - Wrist", KaMAN_k_WristPID_Gx[E_kI]);
  double L_d_Wrist   = frc::SmartDashboard::GetNumber("D Gain - Wrist", KaMAN_k_WristPID_Gx[E_kD]);
  double L_iz_Wrist  = frc::SmartDashboard::GetNumber("I Zone - Wrist", KaMAN_k_WristPID_Gx[E_kIz]);
  double L_max_Wrist = frc::SmartDashboard::GetNumber("Max Output - Wrist", KaMAN_k_WristPID_Gx[E_kMaxOutput]);
  double L_min_Wrist = frc::SmartDashboard::GetNumber("Min Output - Wrist", KaMAN_k_WristPID_Gx[E_kMinOutput]);

  double L_p_Gripper   = frc::SmartDashboard::GetNumber("P Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kP]);
  double L_i_Gripper   = frc::SmartDashboard::GetNumber("I Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kI]);
  double L_d_Gripper   = frc::SmartDashboard::GetNumber("D Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kD]);
  double L_iz_Gripper  = frc::SmartDashboard::GetNumber("I Zone - Gripper", KaMAN_k_GripperPID_Gx[E_kIz]);
  double L_max_Gripper = frc::SmartDashboard::GetNumber("Max Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMaxOutput]);
  double L_min_Gripper = frc::SmartDashboard::GetNumber("Min Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMinOutput]);

  double L_p_Intake   = frc::SmartDashboard::GetNumber("P Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kP]);
  double L_i_Intake   = frc::SmartDashboard::GetNumber("I Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kI]);
  double L_d_Intake   = frc::SmartDashboard::GetNumber("D Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kD]);
  double L_iz_Intake  = frc::SmartDashboard::GetNumber("I Zone - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kIz]);
  double L_max_Intake = frc::SmartDashboard::GetNumber("Max Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]);
  double L_min_Intake = frc::SmartDashboard::GetNumber("Min Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMinOutput]);

  double L_p_Turret   = frc::SmartDashboard::GetNumber("P Gain - Turret", KaMAN_k_TurretPID_Gx[E_P_Gx]);
  double L_i_Turret   = frc::SmartDashboard::GetNumber("I Gain - Turret", KaMAN_k_TurretPID_Gx[E_I_Gx]);
  double L_d_Turret   = frc::SmartDashboard::GetNumber("D Gain - Turret", KaMAN_k_TurretPID_Gx[E_D_Gx]);
  double L_iu_Turret  = frc::SmartDashboard::GetNumber("I Upper - Turret", KaMAN_k_TurretPID_Gx[E_I_Ul]);
  double L_il_Turret  = frc::SmartDashboard::GetNumber("I Lower - Turret", KaMAN_k_TurretPID_Gx[E_I_Ll]);
  double L_max_Turret = frc::SmartDashboard::GetNumber("Max Output - Turret", KaMAN_k_TurretPID_Gx[E_Max_Ul]);
  double L_min_Turret = frc::SmartDashboard::GetNumber("Min Output - Turret", KaMAN_k_TurretPID_Gx[E_Max_Ll]);

  double L_p_Linear   = frc::SmartDashboard::GetNumber("P Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_P_Gx]);
  double L_i_Linear   = frc::SmartDashboard::GetNumber("I Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Gx]);
  double L_d_Linear   = frc::SmartDashboard::GetNumber("D Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_D_Gx]);
  double L_iu_Linear  = frc::SmartDashboard::GetNumber("I Upper - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ul]);
  double L_il_Linear  = frc::SmartDashboard::GetNumber("I Lower - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ll]);
  double L_max_Linear = frc::SmartDashboard::GetNumber("Max Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ul]);
  double L_min_Linear = frc::SmartDashboard::GetNumber("Min Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Gripper] = frc::SmartDashboard::GetNumber("Set Position Gripper", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Wrist] = frc::SmartDashboard::GetNumber("Set Position Wrist", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_LinearSlide] = frc::SmartDashboard::GetNumber("Set Position Linear Slide", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_ArmPivot] = frc::SmartDashboard::GetNumber("Set Position Arm Pivot", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Turret] = frc::SmartDashboard::GetNumber("Set Position Turret", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_IntakeRollers] = frc::SmartDashboard::GetNumber("Set Speed Intake Rollers ", 0);
  LeMAN_b_IntakePosition = frc::SmartDashboard::GetBoolean("Set Position Intake", false);
  if (LeMAN_b_IntakePosition == true)
   {
   VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm] = E_MotorExtend;
   }
  else
   {
   VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm] = E_MotorRetract;
   }
  
  if(L_p_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kP])   { m_ArmPivotPID.SetP(L_p_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kP] = L_p_Pivot; }
  if(L_i_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kI])   { m_ArmPivotPID.SetI(L_i_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kI] = L_i_Pivot; }
  if(L_d_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kD])   { m_ArmPivotPID.SetD(L_d_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kD] = L_d_Pivot; }
  if(L_iz_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kIz]) { m_ArmPivotPID.SetIZone(L_iz_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kIz] = L_iz_Pivot; }
  if((L_max_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]) || (L_min_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kMinOutput])) { m_ArmPivotPID.SetOutputRange(L_min_Pivot, L_max_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kMinOutput] = L_min_Pivot; VaMAN_k_ArmPivotPID_Gx[E_kMaxOutput] = L_max_Pivot; }

  if(L_p_Wrist != VaMAN_k_WristPID_Gx[E_kP])   { m_WristPID.SetP(L_p_Wrist); VaMAN_k_WristPID_Gx[E_kP] = L_p_Wrist; }
  if(L_i_Wrist != VaMAN_k_WristPID_Gx[E_kI])   { m_WristPID.SetI(L_i_Wrist); VaMAN_k_WristPID_Gx[E_kI] = L_i_Wrist; }
  if(L_d_Wrist != VaMAN_k_WristPID_Gx[E_kD])   { m_WristPID.SetD(L_d_Wrist); VaMAN_k_WristPID_Gx[E_kD] = L_d_Wrist; }
  if(L_iz_Wrist != VaMAN_k_WristPID_Gx[E_kIz]) { m_WristPID.SetIZone(L_iz_Wrist); VaMAN_k_WristPID_Gx[E_kIz] = L_iz_Wrist; }
  if((L_max_Wrist != VaMAN_k_WristPID_Gx[E_kMaxOutput]) || (L_min_Wrist != VaMAN_k_WristPID_Gx[E_kMinOutput])) { m_WristPID.SetOutputRange(L_min_Wrist, L_max_Wrist); VaMAN_k_WristPID_Gx[E_kMinOutput] = L_min_Wrist; VaMAN_k_WristPID_Gx[E_kMaxOutput] = L_max_Wrist; }

  if(L_p_Gripper != VaMAN_k_GripperPID_Gx[E_kP])   { m_GripperPID.SetP(L_p_Gripper); VaMAN_k_GripperPID_Gx[E_kP] = L_p_Gripper; }
  if(L_i_Gripper != VaMAN_k_GripperPID_Gx[E_kI])   { m_GripperPID.SetI(L_i_Gripper); VaMAN_k_GripperPID_Gx[E_kI] = L_i_Gripper; }
  if(L_d_Gripper != VaMAN_k_GripperPID_Gx[E_kD])   { m_GripperPID.SetD(L_d_Gripper); VaMAN_k_GripperPID_Gx[E_kD] = L_d_Gripper; }
  if(L_iz_Gripper != VaMAN_k_GripperPID_Gx[E_kIz]) { m_GripperPID.SetIZone(L_iz_Gripper); VaMAN_k_GripperPID_Gx[E_kIz] = L_iz_Gripper; }
  if((L_max_Gripper != VaMAN_k_GripperPID_Gx[E_kMaxOutput]) || (L_min_Gripper != VaMAN_k_GripperPID_Gx[E_kMinOutput])) { m_GripperPID.SetOutputRange(L_min_Gripper, L_max_Gripper); VaMAN_k_GripperPID_Gx[E_kMinOutput] = L_min_Gripper; VaMAN_k_GripperPID_Gx[E_kMaxOutput] = L_max_Gripper; }

  if(L_p_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kP])   { m_IntakeRollersPID.SetP(L_p_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kP] = L_p_Intake; }
  if(L_i_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kI])   { m_IntakeRollersPID.SetI(L_i_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kI] = L_i_Intake; }
  if(L_d_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kD])   { m_IntakeRollersPID.SetD(L_d_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kD] = L_d_Intake; }
  if(L_iz_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kIz]) { m_IntakeRollersPID.SetIZone(L_iz_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kIz] = L_iz_Intake; }
  if((L_max_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]) || (L_min_Pivot != VaMAN_k_IntakeRollersPID_Gx[E_kMinOutput])) { m_IntakeRollersPID.SetOutputRange(L_min_Pivot, L_max_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kMinOutput] = L_min_Pivot; VaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput] = L_max_Intake; }

  if(L_p_Turret != VaMAN_k_TurretPID_Gx[E_P_Gx])   { VaMAN_k_TurretPID_Gx[E_P_Gx] = L_p_Turret; }
  if(L_i_Turret != VaMAN_k_TurretPID_Gx[E_I_Gx])   { VaMAN_k_TurretPID_Gx[E_I_Gx] = L_i_Turret; }
  if(L_d_Turret != VaMAN_k_TurretPID_Gx[E_D_Gx])   { VaMAN_k_TurretPID_Gx[E_D_Gx] = L_d_Turret; }
  if(L_iu_Turret != VaMAN_k_TurretPID_Gx[E_I_Ul]) { VaMAN_k_TurretPID_Gx[E_I_Ul] = L_iu_Turret; }
  if(L_il_Turret != VaMAN_k_TurretPID_Gx[E_I_Ll]) { VaMAN_k_TurretPID_Gx[E_I_Ll] = L_il_Turret; }
  if((L_max_Turret != VaMAN_k_TurretPID_Gx[E_Max_Ul]) || (L_min_Turret != VaMAN_k_TurretPID_Gx[E_Max_Ll])) { VaMAN_k_TurretPID_Gx[E_Max_Ll] = L_min_Turret; VaMAN_k_TurretPID_Gx[E_Max_Ul] = L_max_Turret; }

  if(L_p_Linear != VaMAN_k_LinearSlidePID_Gx[E_P_Gx])   { VaMAN_k_LinearSlidePID_Gx[E_P_Gx] = L_p_Linear; }
  if(L_i_Linear != VaMAN_k_LinearSlidePID_Gx[E_I_Gx])   { VaMAN_k_LinearSlidePID_Gx[E_I_Gx] = L_i_Linear; }
  if(L_d_Linear != VaMAN_k_LinearSlidePID_Gx[E_D_Gx])   { VaMAN_k_LinearSlidePID_Gx[E_D_Gx] = L_d_Linear; }
  if(L_iu_Linear != VaMAN_k_LinearSlidePID_Gx[E_I_Ul]) { VaMAN_k_LinearSlidePID_Gx[E_I_Ul] = L_iu_Linear; }
  if(L_il_Linear != VaMAN_k_LinearSlidePID_Gx[E_I_Ll]) { VaMAN_k_LinearSlidePID_Gx[E_I_Ll] = L_il_Linear; }
  if((L_max_Linear != VaMAN_k_LinearSlidePID_Gx[E_Max_Ul]) || (L_min_Linear != VaMAN_k_LinearSlidePID_Gx[E_Max_Ll])) { VaMAN_k_LinearSlidePID_Gx[E_Max_Ll] = L_min_Linear; VaMAN_k_LinearSlidePID_Gx[E_Max_Ul] = L_max_Linear; }

  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Turret] = frc::SmartDashboard::GetNumber("KeMAN_DegS_TurretRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Turret]);
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot] = frc::SmartDashboard::GetNumber("KeMAN_DegS_ArmPivotRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot]);
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide] = frc::SmartDashboard::GetNumber("KeMAN_InS_LinearSlideRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide]);
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist] = frc::SmartDashboard::GetNumber("KeMAN_DegS_WristRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist]);
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Gripper] = frc::SmartDashboard::GetNumber("KeMAN_DegS_GripperRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Gripper]);
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_IntakeRollers] = frc::SmartDashboard::GetNumber("KeMAN_RPMS_IntakeRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_IntakeRollers]);
  #endif
  }

/******************************************************************************
 * Function:     ManipulatorControlInit
 *
 * Description:  Initialization function for the Manipulator controls.
 ******************************************************************************/
void ManipulatorControlInit()
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;

  VeMAN_e_CmndState  = E_MAN_Init;
  VeMAN_e_AttndState = E_MAN_Init;

  VaMAN_Deg_TurretAngleError = 0.0;
  VaMAN_k_TurretAngleIntegral = 0.0;
  VaMAN_In_LinearSlideError = 0.0;
  VaMAN_k_LinearSlideIntegral = 0.0;

  VeMAN_b_CriteriaMet = false;
  VeMAN_b_Paused = false;

  for (LeMAN_i_Index = E_MAN_Turret;
       LeMAN_i_Index < E_MAN_Sz;
       LeMAN_i_Index = TeMAN_e_ManipulatorActuator(int(LeMAN_i_Index) + 1))
    {
      VsMAN_s_Motors.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_Motors.k_MotorRampRate[LeMAN_i_Index] = 0.0;
      VsMAN_s_Motors.k_MotorTestPower[LeMAN_i_Index] = 0.0;
      VsMAN_s_Motors.k_MotorTestValue[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTemp.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTemp.k_MotorRampRate[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTemp.k_MotorTestPower[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTemp.k_MotorTestValue[LeMAN_i_Index] = 0.0;
    }
  }


/******************************************************************************
 * Function:     ManipulatorControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J 
 ******************************************************************************/
void ManipulatorControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;

  for (LeMAN_i_Index = E_MAN_Turret;
       LeMAN_i_Index < E_MAN_Sz;
       LeMAN_i_Index = TeMAN_e_ManipulatorActuator(int(LeMAN_i_Index) + 1))
    {
      VsMAN_s_Motors.k_MotorTestPower[LeMAN_i_Index] = 0.0;
    }
  VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] = E_MotorControlDisabled;

  if (LsCONT_s_DriverInput->b_IntakeArmIn == true)
    {
    VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] = E_MotorRetract;
    }

  if (LsCONT_s_DriverInput->b_IntakeArmOutTest == true)
    {
    VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] = E_MotorExtend;
    }

  if (LsCONT_s_DriverInput->b_IntakeRollersTest == true)
    {
    VsMAN_s_Motors.k_MotorTestPower[E_MAN_IntakeRollers] = KaMAN_k_ManipulatorTestPower[E_MAN_IntakeRollers];
    }


  VsMAN_s_Motors.k_MotorTestPower[E_MAN_Wrist] = LsCONT_s_DriverInput->Pct_WristTest * KaMAN_k_ManipulatorTestPower[E_MAN_Wrist];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_ArmPivot] = LsCONT_s_DriverInput->Pct_ArmPivotTest * KaMAN_k_ManipulatorTestPower[E_MAN_ArmPivot];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_LinearSlide] = LsCONT_s_DriverInput->Pct_LinearSlideTest * KaMAN_k_ManipulatorTestPower[E_MAN_LinearSlide];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_Turret] = LsCONT_s_DriverInput->Pct_TurretTest * KaMAN_k_ManipulatorTestPower[E_MAN_Turret];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_Gripper] = LsCONT_s_DriverInput->pct_IntakeRollerTest * KaMAN_k_ManipulatorTestPower[E_MAN_Gripper];
  }

/******************************************************************************
 * Function:     Update_Command_Attained_State
 *
 * Description:  Updates the commanded and attained states for the manipulator
 ******************************************************************************/
bool Update_Command_Atained_State(bool                    LeMAN_b_CriteriaMet,
                                  TeMAN_ManipulatorStates LeMAN_e_SchedState)
  {
  TeMAN_ManipulatorStates LeMAN_e_CmndState = VeMAN_e_CmndState;

  if(LeMAN_b_CriteriaMet == true)
    {
    VeMAN_e_AttndState = LeMAN_e_CmndState;
    LeMAN_b_CriteriaMet = false;
    }

  if((LeMAN_e_SchedState != VeMAN_e_AttndState) &&
     (VeMAN_e_CmndState  == VeMAN_e_AttndState))
    {
    LeMAN_e_CmndState = KaMAN_e_ControllingTable[LeMAN_e_SchedState][VeMAN_e_AttndState];
    }

  VeMAN_e_CmndState = LeMAN_e_CmndState;

  return(LeMAN_b_CriteriaMet);
  }

/******************************************************************************
 * Function:     CmndStateReached
 *
 * Description:  Checks to see if we have reached the desired commanded state
 ******************************************************************************/
bool CmndStateReached(TeMAN_ManipulatorStates LeMAN_e_CmndState)
  {
  bool LeMAN_b_CriteriaMet = false;

  if((VsMAN_s_Sensors.Deg_Turret <= (KaMAN_Deg_TurretAngle[LeMAN_e_CmndState] + KaMAN_Deg_TurretDb[LeMAN_e_CmndState])) &&
     (VsMAN_s_Sensors.Deg_Turret >= (KaMAN_Deg_TurretAngle[LeMAN_e_CmndState] - KaMAN_Deg_TurretDb[LeMAN_e_CmndState])) &&

     (VsMAN_s_Sensors.Deg_ArmPivot <= (KaMAN_Deg_ArmPivotAngle[LeMAN_e_CmndState] + KaMAN_Deg_ArmPivotDb[LeMAN_e_CmndState])) &&
     (VsMAN_s_Sensors.Deg_ArmPivot >= (KaMAN_Deg_ArmPivotAngle[LeMAN_e_CmndState] - KaMAN_Deg_ArmPivotDb[LeMAN_e_CmndState])) &&

     (VsMAN_s_Sensors.In_LinearSlide <= (KaMAN_In_LinearSlidePosition[LeMAN_e_CmndState] + KaMAN_In_LinearSlideDb[LeMAN_e_CmndState])) &&
     (VsMAN_s_Sensors.In_LinearSlide >= (KaMAN_In_LinearSlidePosition[LeMAN_e_CmndState] - KaMAN_In_LinearSlideDb[LeMAN_e_CmndState])) &&

     (VsMAN_s_Sensors.Deg_Wrist <= (KaMAN_Deg_WristAngle[LeMAN_e_CmndState] + KaMAN_Deg_WristDb[LeMAN_e_CmndState])) &&
     (VsMAN_s_Sensors.Deg_Wrist >= (KaMAN_Deg_WristAngle[LeMAN_e_CmndState] - KaMAN_Deg_WristDb[LeMAN_e_CmndState])) &&

     (VsMAN_s_Sensors.RPM_IntakeRollers <= (KaMAN_RPM_IntakeSpeed[LeMAN_e_CmndState] + KaMAN_RPM_IntakeSpeedDb[LeMAN_e_CmndState])) &&
     (VsMAN_s_Sensors.RPM_IntakeRollers >= (KaMAN_RPM_IntakeSpeed[LeMAN_e_CmndState] - KaMAN_RPM_IntakeSpeedDb[LeMAN_e_CmndState])) &&

     ((VsMAN_s_Sensors.b_IntakeArmExtended == true && KaMAN_e_IntakePneumatics[LeMAN_e_CmndState] == E_MotorExtend) ||
      (VsMAN_s_Sensors.b_IntakeArmExtended == false && KaMAN_e_IntakePneumatics[LeMAN_e_CmndState] == E_MotorRetract)))
      {
      LeMAN_b_CriteriaMet = true;
      }

  return(LeMAN_b_CriteriaMet);
  }

/******************************************************************************
 * Function:     UpdateManipulatorActuators
 *
 * Description:  Updates the intermediate state of the actuartors for the 
 *               manipulator
 ******************************************************************************/
void UpdateManipulatorActuators(TeMAN_ManipulatorStates LeMAN_e_CmndState)
  {
   /* Ok, let's set the desired postions: */
   
   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Turret] = RampTo(KaMAN_Deg_TurretAngle[LeMAN_e_CmndState] / KeENC_k_TurretEncoderScaler, 
                                                         VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Turret],
                                                         KeMAN_DegS_TurretRate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot] = RampTo(KaMAN_Deg_ArmPivotAngle[LeMAN_e_CmndState] / KeENC_k_ArmPivot, 
                                                           VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot],
                                                           KeMAN_DegS_ArmPivotRate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide] = RampTo(KaMAN_In_LinearSlidePosition[LeMAN_e_CmndState] / KeENC_k_LinearSlideEncoderScaler, 
                                                              VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                              KeMAN_InS_LinearSlideRate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist] = RampTo(KaMAN_Deg_WristAngle[LeMAN_e_CmndState] / KeENC_Deg_Wrist, 
                                                        VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist],
                                                        KeMAN_DegS_WristRate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers] = RampTo(KaMAN_RPM_IntakeSpeed[LeMAN_e_CmndState] / KeENC_RPM_IntakeRollers, 
                                                                VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers],
                                                                KeMAN_RPMS_IntakeRate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeArm] = KaMAN_e_IntakePneumatics[LeMAN_e_CmndState];
  }

/******************************************************************************
 * Function:     UpdateGripperActuator
 *
 * Description:  Updates the gripper roller control
 ******************************************************************************/
void UpdateGripperActuator(TeMAN_ManipulatorStates LeMAN_e_CmndState,
                                TeMAN_ManipulatorStates LeMAN_e_AttndState,
                                bool                    LeMAN_b_ReleaseObj,
                                bool                    LeMAN_b_ObjDetected)
  {
   double LeMAN_k_TempCmnd = 0.0;
   bool   LeMAN_b_AllowedReleaseState = false;

   if ((LeMAN_e_AttndState == LeMAN_e_CmndState) &&

       ((LeMAN_e_AttndState == E_MAN_PositioningHighCube) ||
        (LeMAN_e_AttndState == E_MAN_PositioningLowCube)  ||
        (LeMAN_e_AttndState == E_MAN_PositioningHighCone)  ||
        (LeMAN_e_AttndState == E_MAN_PositioningLowCone) ||
        (LeMAN_e_AttndState == E_MAN_MainIntake) ||
        (LeMAN_e_AttndState == E_MAN_FloorIntake)))
     {
      LeMAN_b_AllowedReleaseState = true;
     }

   if ((LeMAN_b_ReleaseObj == true) && 
       (LeMAN_b_AllowedReleaseState == true))
     {
     LeMAN_k_TempCmnd = KeMAN_k_GripperRelease;
     }
   else if ((LeMAN_b_ObjDetected == false) &&
            (((LeMAN_e_AttndState == E_MAN_MainIntake) && (LeMAN_e_CmndState  == E_MAN_MainIntake)) ||
             ((LeMAN_e_AttndState == E_MAN_FloorIntake) && (LeMAN_e_CmndState  == E_MAN_FloorIntake))))
     {
     LeMAN_k_TempCmnd = KeMAN_k_GripperIntake;
     }

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper] = LeMAN_k_TempCmnd;
  }

/******************************************************************************
 * Function:     ManipulatorControlMain
 *
 * Description:  Main calling function for manipulator control.
 ******************************************************************************/
void ManipulatorControlMain(TeMAN_ManipulatorStates LeMAN_e_SchedState,
                            bool                    LeMAN_b_TestPowerOverride,
                            bool                    LeMAN_b_DropObject)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;
  // LeMAN_b_TestPowerOverride = false;

  if (LeMAN_b_TestPowerOverride == true)
    {
    // Do nothing.  Robot is in test state using power commands for all the acutators
    }
  else if (VeMAN_b_TestState == true)
    {
    /* Only used for testing/calibration. */
    for (LeMAN_i_Index = E_MAN_Turret;
         LeMAN_i_Index < E_MAN_Sz;
         LeMAN_i_Index = TeMAN_e_ManipulatorActuator(int(LeMAN_i_Index) + 1))
      {
      VsMAN_s_MotorsTemp.k_MotorCmnd[LeMAN_i_Index] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[LeMAN_i_Index] / VaMAN_k_PositionToEncoder[LeMAN_i_Index], 
                                                             VsMAN_s_MotorsTemp.k_MotorCmnd[LeMAN_i_Index],
                                                             VsMAN_s_MotorsTest.k_MotorRampRate[LeMAN_i_Index]);
      }
     VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm] = VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm];
    }
  else
    {
    /* This is the actual manipulator control */

    VeMAN_b_CriteriaMet = Update_Command_Atained_State(VeMAN_b_CriteriaMet,
                                                       LeMAN_e_SchedState);

    if ((LeMAN_e_SchedState != VeMAN_e_CmndState) ||
        (LeMAN_e_SchedState != VeMAN_e_AttndState))
      {
        UpdateManipulatorActuators(VeMAN_e_CmndState);

        VeMAN_b_CriteriaMet = CmndStateReached(VeMAN_e_CmndState);
      }
    else
      {
        UpdateManipulatorActuators(VeMAN_e_CmndState);
      }


    UpdateGripperActuator(VeMAN_e_CmndState,
                          VeMAN_e_AttndState,
                          LeMAN_b_DropObject,
                          false);  // Need to come up with object detected
    }
    
    /* Final output to the motor command that will be sent to the motor controller: */
    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Turret] =  Control_PID( VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Turret],
                                                             VsMAN_s_Motors.k_MotorCmnd[E_MAN_Turret],
                                                            &VaMAN_Deg_TurretAngleError,
                                                            &VaMAN_k_TurretAngleIntegral,
                                                             VaMAN_k_TurretPID_Gx[E_P_Gx],
                                                             VaMAN_k_TurretPID_Gx[E_I_Gx],
                                                             VaMAN_k_TurretPID_Gx[E_D_Gx],
                                                             VaMAN_k_TurretPID_Gx[E_P_Ul],
                                                             VaMAN_k_TurretPID_Gx[E_P_Ll],
                                                             VaMAN_k_TurretPID_Gx[E_I_Ul],
                                                             VaMAN_k_TurretPID_Gx[E_I_Ll],
                                                             VaMAN_k_TurretPID_Gx[E_D_Ul],
                                                             VaMAN_k_TurretPID_Gx[E_D_Ll],
                                                             VaMAN_k_TurretPID_Gx[E_Max_Ul],
                                                             VaMAN_k_TurretPID_Gx[E_Max_Ll]);

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_ArmPivot] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_LinearSlide] =  Control_PID( VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                                  VsMAN_s_Motors.k_MotorCmnd[E_MAN_LinearSlide],
                                                                 &VaMAN_In_LinearSlideError,
                                                                 &VaMAN_k_LinearSlideIntegral,
                                                                  VaMAN_k_LinearSlidePID_Gx[E_P_Gx],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_I_Gx],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_D_Gx],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_P_Ul],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_P_Ll],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_I_Ul],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_I_Ll],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_D_Ul],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_D_Ll],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_Max_Ul],
                                                                  VaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Wrist] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Gripper] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_IntakeRollers] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers];

    VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] = VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm];
  }