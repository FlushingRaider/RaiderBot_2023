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

// the rpm where under it we consider the gripper holding something
double                  C_GripperRPMHoldingThreshold = 0.0;
double                  C_GripperRPMReadyThreshold = 1.0;
bool                    VeMAN_b_ReadyToGrab;
bool                    VeMAN_b_HasObject;

TeMAN_ManipulatorStates VeMAN_e_CmndState  = E_MAN_Init; // What is our next/current step?
TeMAN_ManipulatorStates VeMAN_e_AttndState = E_MAN_Init; // What is our desired end state?

TeMAN_MotorControl      VsMAN_s_Motors; // All of the motor commands for the manipulator/intake motors
TeMAN_MotorControl      VsMAN_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeMAN_MotorControl      VsMAN_s_MotorsTest; // Temporary commands for the motors, not the final output
TsMAN_Sensor            VsMAN_s_Sensors; // All of the sensor values for the manipulator/intake motors

double                  VeMAN_t_TransitionTime = 0;
double                  VeMAN_t_GripperHoldTime = 0; // Amount of time after object is detected, to continue pulling in
double                  VaMAN_In_LinearSlideError;
double                  VaMAN_k_LinearSlideIntegral;

double                  VaMAN_k_ArmPivotPID_Gx[E_PID_SparkMaxCalSz];
double                  VaMAN_k_WristPID_Gx[E_PID_SparkMaxCalSz];
double                  VaMAN_k_GripperPID_Gx[E_PID_SparkMaxCalSz];
double                  VaMAN_k_IntakeRollersPID_Gx[E_PID_SparkMaxCalSz];
double                  VaMAN_k_LinearSlidePID_Gx[E_PID_CalSz];

bool                    VeMAN_b_CriteriaMet = false;
bool                    VeMAN_b_ConeHold = false;  // Used for the holding power.  If cone, use cone cal, otherwise use cube.

#ifdef Manipulator_Test
bool                    VeMAN_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeMAN_b_TestState = false;
#endif

/******************************************************************************
 * Function:     ManipulatorMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the Arm and intake motors.
 ******************************************************************************/
void ManipulatorMotorConfigsInit(rev::SparkMaxPIDController m_ArmPivotPID,
                                 rev::SparkMaxPIDController m_WristPID,
                                 rev::SparkMaxPIDController m_GripperPID)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;
  T_PID_Cal LeMAN_i_Index3 = E_P_Gx;

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



  for (LeMAN_i_Index = E_MAN_ArmPivot;
       LeMAN_i_Index < E_MAN_Sz;
       LeMAN_i_Index = TeMAN_e_ManipulatorActuator(int(LeMAN_i_Index) + 1))
    {
      VsMAN_s_Motors.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTemp.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_MotorsTest.k_MotorCmnd[LeMAN_i_Index] = 0.0;
    }



  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot] = KaMAN_DegS_ArmPivotRate[E_MAN_Init][E_MAN_Init];
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide] = KaMAN_InS_LinearSlideRate[E_MAN_Init][E_MAN_Init];
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist] = KaMAN_DegS_WristRate[E_MAN_Init][E_MAN_Init];
  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Gripper] = 1.0;

  VaMAN_In_LinearSlideError = 0.0;
  VaMAN_k_LinearSlideIntegral = 0.0;

  for (LeMAN_i_Index3 = E_P_Gx;
       LeMAN_i_Index3 < E_PID_CalSz;
       LeMAN_i_Index3 = T_PID_Cal(int(LeMAN_i_Index3) + 1))
    {
    VaMAN_k_LinearSlidePID_Gx[LeMAN_i_Index3] = KaMAN_k_LinearSlidePID_Gx[LeMAN_i_Index3];
    }

  #ifdef Manipulator_Test
  T_PID_SparkMaxCal LeMAN_i_Index2 = E_kP;

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
    VaMAN_k_LinearSlidePID_Gx[LeMAN_i_Index3] = KaMAN_k_LinearSlidePID_Gx[LeMAN_i_Index3];
    }

  // display PID coefficients on SmartDashboard
  // frc::SmartDashboard::PutNumber("P Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Max Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMinOutput]);

  // frc::SmartDashboard::PutNumber("P Gain - Wrist", KaMAN_k_WristPID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain - Wrist", KaMAN_k_WristPID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain - Wrist", KaMAN_k_WristPID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone - Wrist", KaMAN_k_WristPID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Max Output - Wrist", KaMAN_k_WristPID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output - Wrist", KaMAN_k_WristPID_Gx[E_kMinOutput]);

  // frc::SmartDashboard::PutNumber("P Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone - Gripper", KaMAN_k_GripperPID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Max Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMinOutput]);

  // frc::SmartDashboard::PutNumber("P Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Max Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMinOutput]);

  // frc::SmartDashboard::PutNumber("P Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_P_Gx]);
  // frc::SmartDashboard::PutNumber("I Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Gx]);
  // frc::SmartDashboard::PutNumber("D Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_D_Gx]);
  // frc::SmartDashboard::PutNumber("I Upper - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ul]);
  // frc::SmartDashboard::PutNumber("I Lower - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ll]);
  // frc::SmartDashboard::PutNumber("Max Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ul]);
  // frc::SmartDashboard::PutNumber("Min Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KaMAN_DegS_ArmPivotRate", KaMAN_DegS_ArmPivotRate[E_MAN_Init][E_MAN_Init]);
  frc::SmartDashboard::PutNumber("KaMAN_InS_LinearSlideRate", KaMAN_InS_LinearSlideRate[E_MAN_Init][E_MAN_Init]);
  frc::SmartDashboard::PutNumber("KaMAN_DegS_WristRate", [E_MAN_Init][E_MAN_Init]);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Set Position Gripper", 0);
  frc::SmartDashboard::PutNumber("Set Position Wrist", 0);
  frc::SmartDashboard::PutNumber("Set Position Linear Slide", 0);
  frc::SmartDashboard::PutNumber("Set Position Arm Pivot", 0);
  frc::SmartDashboard::PutNumber("Set Speed Intake Rollers ", 0);
  frc::SmartDashboard::PutNumber("Set Gripper Power", 0);
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
                                rev::SparkMaxPIDController m_GripperPID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef Manipulator_Test
  bool LeMAN_b_IntakePosition = false;  // false is retracted, true extended
  // double L_p_Pivot   = frc::SmartDashboard::GetNumber("P Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kP]);
  // double L_i_Pivot   = frc::SmartDashboard::GetNumber("I Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kI]);
  // double L_d_Pivot   = frc::SmartDashboard::GetNumber("D Gain - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kD]);
  // double L_iz_Pivot  = frc::SmartDashboard::GetNumber("I Zone - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kIz]);
  // double L_max_Pivot = frc::SmartDashboard::GetNumber("Max Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]);
  // double L_min_Pivot = frc::SmartDashboard::GetNumber("Min Output - Pivot", KaMAN_k_ArmPivotPID_Gx[E_kMinOutput]);

  // double L_p_Wrist   = frc::SmartDashboard::GetNumber("P Gain - Wrist", KaMAN_k_WristPID_Gx[E_kP]);
  // double L_i_Wrist   = frc::SmartDashboard::GetNumber("I Gain - Wrist", KaMAN_k_WristPID_Gx[E_kI]);
  // double L_d_Wrist   = frc::SmartDashboard::GetNumber("D Gain - Wrist", KaMAN_k_WristPID_Gx[E_kD]);
  // double L_iz_Wrist  = frc::SmartDashboard::GetNumber("I Zone - Wrist", KaMAN_k_WristPID_Gx[E_kIz]);
  // double L_max_Wrist = frc::SmartDashboard::GetNumber("Max Output - Wrist", KaMAN_k_WristPID_Gx[E_kMaxOutput]);
  // double L_min_Wrist = frc::SmartDashboard::GetNumber("Min Output - Wrist", KaMAN_k_WristPID_Gx[E_kMinOutput]);

  // double L_p_Gripper   = frc::SmartDashboard::GetNumber("P Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kP]);
  // double L_i_Gripper   = frc::SmartDashboard::GetNumber("I Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kI]);
  // double L_d_Gripper   = frc::SmartDashboard::GetNumber("D Gain - Gripper", KaMAN_k_GripperPID_Gx[E_kD]);
  // double L_iz_Gripper  = frc::SmartDashboard::GetNumber("I Zone - Gripper", KaMAN_k_GripperPID_Gx[E_kIz]);
  // double L_max_Gripper = frc::SmartDashboard::GetNumber("Max Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMaxOutput]);
  // double L_min_Gripper = frc::SmartDashboard::GetNumber("Min Output - Gripper", KaMAN_k_GripperPID_Gx[E_kMinOutput]);

  // double L_p_Intake   = frc::SmartDashboard::GetNumber("P Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kP]);
  // double L_i_Intake   = frc::SmartDashboard::GetNumber("I Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kI]);
  // double L_d_Intake   = frc::SmartDashboard::GetNumber("D Gain - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kD]);
  // double L_iz_Intake  = frc::SmartDashboard::GetNumber("I Zone - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kIz]);
  // double L_max_Intake = frc::SmartDashboard::GetNumber("Max Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]);
  // double L_min_Intake = frc::SmartDashboard::GetNumber("Min Output - Intake", KaMAN_k_IntakeRollersPID_Gx[E_kMinOutput]);

  // double L_p_Linear   = frc::SmartDashboard::GetNumber("P Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_P_Gx]);
  // double L_i_Linear   = frc::SmartDashboard::GetNumber("I Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Gx]);
  // double L_d_Linear   = frc::SmartDashboard::GetNumber("D Gain - Linear", KaMAN_k_LinearSlidePID_Gx[E_D_Gx]);
  // double L_iu_Linear  = frc::SmartDashboard::GetNumber("I Upper - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ul]);
  // double L_il_Linear  = frc::SmartDashboard::GetNumber("I Lower - Linear", KaMAN_k_LinearSlidePID_Gx[E_I_Ll]);
  // double L_max_Linear = frc::SmartDashboard::GetNumber("Max Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ul]);
  // double L_min_Linear = frc::SmartDashboard::GetNumber("Min Output - Linear", KaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Gripper] = frc::SmartDashboard::GetNumber("Set Position Gripper", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Wrist] = frc::SmartDashboard::GetNumber("Set Position Wrist", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_LinearSlide] = frc::SmartDashboard::GetNumber("Set Position Linear Slide", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_ArmPivot] = frc::SmartDashboard::GetNumber("Set Position Arm Pivot", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_IntakeRollers] = frc::SmartDashboard::GetNumber("Set Speed Intake Rollers ", 0);
  VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Gripper] = frc::SmartDashboard::GetNumber("Set Gripper Power", 0);

  LeMAN_b_IntakePosition = frc::SmartDashboard::GetBoolean("Set Position Intake", false);
  if (LeMAN_b_IntakePosition == true)
   {
   VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm] = E_MotorExtend;
   }
  else
   {
   VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm] = E_MotorRetract;
   }
  
  // if(L_p_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kP])   { m_ArmPivotPID.SetP(L_p_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kP] = L_p_Pivot; }
  // if(L_i_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kI])   { m_ArmPivotPID.SetI(L_i_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kI] = L_i_Pivot; }
  // if(L_d_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kD])   { m_ArmPivotPID.SetD(L_d_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kD] = L_d_Pivot; }
  // if(L_iz_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kIz]) { m_ArmPivotPID.SetIZone(L_iz_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kIz] = L_iz_Pivot; }
  // if((L_max_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kMaxOutput]) || (L_min_Pivot != VaMAN_k_ArmPivotPID_Gx[E_kMinOutput])) { m_ArmPivotPID.SetOutputRange(L_min_Pivot, L_max_Pivot); VaMAN_k_ArmPivotPID_Gx[E_kMinOutput] = L_min_Pivot; VaMAN_k_ArmPivotPID_Gx[E_kMaxOutput] = L_max_Pivot; }

  // if(L_p_Wrist != VaMAN_k_WristPID_Gx[E_kP])   { m_WristPID.SetP(L_p_Wrist); VaMAN_k_WristPID_Gx[E_kP] = L_p_Wrist; }
  // if(L_i_Wrist != VaMAN_k_WristPID_Gx[E_kI])   { m_WristPID.SetI(L_i_Wrist); VaMAN_k_WristPID_Gx[E_kI] = L_i_Wrist; }
  // if(L_d_Wrist != VaMAN_k_WristPID_Gx[E_kD])   { m_WristPID.SetD(L_d_Wrist); VaMAN_k_WristPID_Gx[E_kD] = L_d_Wrist; }
  // if(L_iz_Wrist != VaMAN_k_WristPID_Gx[E_kIz]) { m_WristPID.SetIZone(L_iz_Wrist); VaMAN_k_WristPID_Gx[E_kIz] = L_iz_Wrist; }
  // if((L_max_Wrist != VaMAN_k_WristPID_Gx[E_kMaxOutput]) || (L_min_Wrist != VaMAN_k_WristPID_Gx[E_kMinOutput])) { m_WristPID.SetOutputRange(L_min_Wrist, L_max_Wrist); VaMAN_k_WristPID_Gx[E_kMinOutput] = L_min_Wrist; VaMAN_k_WristPID_Gx[E_kMaxOutput] = L_max_Wrist; }

  // if(L_p_Gripper != VaMAN_k_GripperPID_Gx[E_kP])   { m_GripperPID.SetP(L_p_Gripper); VaMAN_k_GripperPID_Gx[E_kP] = L_p_Gripper; }
  // if(L_i_Gripper != VaMAN_k_GripperPID_Gx[E_kI])   { m_GripperPID.SetI(L_i_Gripper); VaMAN_k_GripperPID_Gx[E_kI] = L_i_Gripper; }
  // if(L_d_Gripper != VaMAN_k_GripperPID_Gx[E_kD])   { m_GripperPID.SetD(L_d_Gripper); VaMAN_k_GripperPID_Gx[E_kD] = L_d_Gripper; }
  // if(L_iz_Gripper != VaMAN_k_GripperPID_Gx[E_kIz]) { m_GripperPID.SetIZone(L_iz_Gripper); VaMAN_k_GripperPID_Gx[E_kIz] = L_iz_Gripper; }
  // if((L_max_Gripper != VaMAN_k_GripperPID_Gx[E_kMaxOutput]) || (L_min_Gripper != VaMAN_k_GripperPID_Gx[E_kMinOutput])) { m_GripperPID.SetOutputRange(L_min_Gripper, L_max_Gripper); VaMAN_k_GripperPID_Gx[E_kMinOutput] = L_min_Gripper; VaMAN_k_GripperPID_Gx[E_kMaxOutput] = L_max_Gripper; }

  // if(L_p_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kP])   { m_IntakeRollersPID.SetP(L_p_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kP] = L_p_Intake; }
  // if(L_i_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kI])   { m_IntakeRollersPID.SetI(L_i_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kI] = L_i_Intake; }
  // if(L_d_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kD])   { m_IntakeRollersPID.SetD(L_d_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kD] = L_d_Intake; }
  // if(L_iz_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kIz]) { m_IntakeRollersPID.SetIZone(L_iz_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kIz] = L_iz_Intake; }
  // if((L_max_Intake != VaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput]) || (L_min_Pivot != VaMAN_k_IntakeRollersPID_Gx[E_kMinOutput])) { m_IntakeRollersPID.SetOutputRange(L_min_Pivot, L_max_Intake); VaMAN_k_IntakeRollersPID_Gx[E_kMinOutput] = L_min_Pivot; VaMAN_k_IntakeRollersPID_Gx[E_kMaxOutput] = L_max_Intake; }

  // if(L_p_Linear != VaMAN_k_LinearSlidePID_Gx[E_P_Gx])   { VaMAN_k_LinearSlidePID_Gx[E_P_Gx] = L_p_Linear; }
  // if(L_i_Linear != VaMAN_k_LinearSlidePID_Gx[E_I_Gx])   { VaMAN_k_LinearSlidePID_Gx[E_I_Gx] = L_i_Linear; }
  // if(L_d_Linear != VaMAN_k_LinearSlidePID_Gx[E_D_Gx])   { VaMAN_k_LinearSlidePID_Gx[E_D_Gx] = L_d_Linear; }
  // if(L_iu_Linear != VaMAN_k_LinearSlidePID_Gx[E_I_Ul]) { VaMAN_k_LinearSlidePID_Gx[E_I_Ul] = L_iu_Linear; }
  // if(L_il_Linear != VaMAN_k_LinearSlidePID_Gx[E_I_Ll]) { VaMAN_k_LinearSlidePID_Gx[E_I_Ll] = L_il_Linear; }
  // if((L_max_Linear != VaMAN_k_LinearSlidePID_Gx[E_Max_Ul]) || (L_min_Linear != VaMAN_k_LinearSlidePID_Gx[E_Max_Ll])) { VaMAN_k_LinearSlidePID_Gx[E_Max_Ll] = L_min_Linear; VaMAN_k_LinearSlidePID_Gx[E_Max_Ul] = L_max_Linear; }

  // VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot] = frc::SmartDashboard::GetNumber("KaMAN_DegS_ArmPivotRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot]);
  // VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide] = frc::SmartDashboard::GetNumber("KaMAN_InS_LinearSlideRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide]);
  // VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist] = frc::SmartDashboard::GetNumber("KaMAN_DegS_WristRate", VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist]);
   #endif
 
  // C_GripperRPMHoldingThreshold = frc::SmartDashboard::GetNumber("Holding RPM Threshold", 0.0);
  // C_GripperRPMHoldingThreshold = frc::SmartDashboard::GetNumber("Intake Ready RPM Threshold", 1.0);
  }

/******************************************************************************
 * Function:     ManipulatorControlInit
 *
 * Description:  Initialization function for the Manipulator controls.
 ******************************************************************************/
void ManipulatorControlInit()
  {
  VeMAN_e_CmndState  = E_MAN_Init;
  VeMAN_e_AttndState = E_MAN_Init;

  VaMAN_In_LinearSlideError = 0.0;
  VaMAN_k_LinearSlideIntegral = 0.0;

  VeMAN_b_CriteriaMet = false;
  VeMAN_t_TransitionTime = 0.0;
  VeMAN_b_ConeHold = false;
  }


/******************************************************************************
 * Function:     ManipulatorControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J 
 ******************************************************************************/
void ManipulatorControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;

  for (LeMAN_i_Index = E_MAN_ArmPivot;
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

  VeMAN_t_TransitionTime += C_ExeTime;

  if((VeMAN_t_TransitionTime >= KeMAN_t_StateTimeOut) ||

     ((VsMAN_s_Sensors.Deg_ArmPivot <= (KaMAN_Deg_ArmPivotAngle[LeMAN_e_CmndState] + KaMAN_Deg_ArmPivotDb[LeMAN_e_CmndState])) &&
      (VsMAN_s_Sensors.Deg_ArmPivot >= (KaMAN_Deg_ArmPivotAngle[LeMAN_e_CmndState] - KaMAN_Deg_ArmPivotDb[LeMAN_e_CmndState])) &&

      (VsMAN_s_Sensors.In_LinearSlide <= (KaMAN_In_LinearSlidePosition[LeMAN_e_CmndState] + KaMAN_In_LinearSlideDb[LeMAN_e_CmndState])) &&
      (VsMAN_s_Sensors.In_LinearSlide >= (KaMAN_In_LinearSlidePosition[LeMAN_e_CmndState] - KaMAN_In_LinearSlideDb[LeMAN_e_CmndState])) &&

      (VsMAN_s_Sensors.Deg_Wrist <= (KaMAN_Deg_WristAngle[LeMAN_e_CmndState] + KaMAN_Deg_WristDb[LeMAN_e_CmndState])) &&
      (VsMAN_s_Sensors.Deg_Wrist >= (KaMAN_Deg_WristAngle[LeMAN_e_CmndState] - KaMAN_Deg_WristDb[LeMAN_e_CmndState])) &&

      ((VsMAN_s_Sensors.b_IntakeArmExtended == true && KaMAN_e_IntakePneumatics[LeMAN_e_CmndState] == E_MotorExtend) ||
       (VsMAN_s_Sensors.b_IntakeArmExtended == false && KaMAN_e_IntakePneumatics[LeMAN_e_CmndState] == E_MotorRetract))))
      {
      LeMAN_b_CriteriaMet = true;
      VeMAN_t_TransitionTime = 0.0;
      }

  return(LeMAN_b_CriteriaMet);
  }

/******************************************************************************
 * Function:     UpdateManipulatorActuators
 *
 * Description:  Updates the intermediate state of the actuartors for the 
 *               manipulator
 ******************************************************************************/
void UpdateManipulatorActuators(TeMAN_ManipulatorStates LeMAN_e_CmndState,
                                TeMAN_ManipulatorStates LeMAN_e_AttndState)
  {
   double    LeMAN_InS_LinearSlideRate = 0.0;
   double    LeMAN_k_ArmPivotRate = 0.0;
   double    LeMAN_DegS_WristRate = 0.0;
   T_PID_Cal LeMAN_i_Index = E_P_Gx;

   LeMAN_k_ArmPivotRate = KaMAN_DegS_ArmPivotRate[LeMAN_e_CmndState][LeMAN_e_AttndState];

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot] = RampTo(KaMAN_Deg_ArmPivotAngle[LeMAN_e_CmndState] / KeENC_k_ArmPivot, 
                                                           VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot],
                                                           LeMAN_k_ArmPivotRate);

   LeMAN_InS_LinearSlideRate = KaMAN_InS_LinearSlideRate[LeMAN_e_CmndState][LeMAN_e_AttndState];

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide] = RampTo(KaMAN_In_LinearSlidePosition[LeMAN_e_CmndState],  // / KeENC_k_LinearSlideEncoderScaler
                                                              VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                              LeMAN_InS_LinearSlideRate);

   LeMAN_DegS_WristRate = KaMAN_DegS_WristRate[LeMAN_e_CmndState][LeMAN_e_AttndState];

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist] = RampTo(KaMAN_Deg_WristAngle[LeMAN_e_CmndState] / KeENC_Deg_Wrist, 
                                                        VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist],
                                                        LeMAN_DegS_WristRate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers] = KaMAN_RPM_IntakePower[LeMAN_e_CmndState]; // Change to power based.  Don't think we need speed control here...

   VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm] = KaMAN_e_IntakePneumatics[LeMAN_e_CmndState];
  }

/******************************************************************************
 * Function:     UpdateGripperActuator
 *
 * Description:  Updates the gripper roller control
 ******************************************************************************/
void UpdateGripperActuator(TeMAN_ManipulatorStates LeMAN_e_CmndState,
                           TeMAN_ManipulatorStates LeMAN_e_AttndState,
                           bool                    LeMAN_b_DropObjectSlow,
                           bool                    LeMAN_b_DropObjectFast)
  {
   double LeMAN_k_TempCmnd = 0.0;
   bool   LeMAN_b_AllowedReleaseState = false;
   bool   LeMAN_b_ConeState = false;
   bool   LeMAN_b_CubeState = false;

  // frc::SmartDashboard::PutBoolean("Gripper Has Object", VeMAN_b_HasObject);
  // frc::SmartDashboard::PutBoolean("Gripper ready to grab", VeMAN_b_ReadyToGrab);

  // if (fabs(VsMAN_s_Sensors.RPM_Gripper) > C_GripperRPMReadyThreshold)
  // {
  //   VeMAN_b_ReadyToGrab = true;
  // }
  // else if ((fabs(VsMAN_s_Sensors.RPM_Gripper) <= C_GripperRPMReadyThreshold) && (VeMAN_b_HasObject == false))
  // {
  //   VeMAN_b_ReadyToGrab = false;
  // }

  // if ((fabs(VsMAN_s_Sensors.RPM_Gripper) <= C_GripperRPMHoldingThreshold) && (VeMAN_b_ReadyToGrab) &&
  //     (LeMAN_e_CmndState == E_MAN_MidCubeIntake ||
  //      LeMAN_e_CmndState == E_MAN_MidConeIntake ||
  //      LeMAN_e_CmndState == E_MAN_FloorConeDrop ||
  //      LeMAN_e_CmndState == E_MAN_MainIntake))
  // {
  //   VeMAN_b_HasObject = true;
  // }
  // else if (fabs(VsMAN_s_Sensors.RPM_Gripper) > C_GripperRPMHoldingThreshold)
  // {
  //   VeMAN_b_HasObject = false;
  // }

   /* Determine if we are attempting to drop a cube or cone: */
   if ((LeMAN_e_AttndState == E_MAN_HighCubeDrop) ||
       (LeMAN_e_AttndState == E_MAN_LowCubeDrop)  ||
       (LeMAN_e_AttndState == E_MAN_MidCubeIntake) ||
       (LeMAN_e_AttndState == E_MAN_MainIntake))
     {
      LeMAN_b_CubeState = true;
      VeMAN_b_ConeHold = false;
     }

   if ((LeMAN_e_AttndState == E_MAN_HighConeDrop) ||
       (LeMAN_e_AttndState == E_MAN_LowConeDrop)  ||
       (LeMAN_e_AttndState == E_MAN_MidConeIntake) ||
       (LeMAN_e_AttndState == E_MAN_FloorConeDrop))
     {
      LeMAN_b_ConeState = true;
      VeMAN_b_ConeHold = true;
     }

   /* Determine if we are in an allowed state to drop: */
   if ((LeMAN_e_AttndState == LeMAN_e_CmndState) &&

       ((LeMAN_b_CubeState == true) ||
        (LeMAN_b_ConeState == true)  ||
        ((LeMAN_e_AttndState == E_MAN_Driving) && (VeMAN_b_ConeHold == false)) ||  // Do not allow a cone to be shot out in driving
        (LeMAN_e_AttndState == E_MAN_MainIntake)))
     {
      LeMAN_b_AllowedReleaseState = true;
     }


   if ((LeMAN_b_DropObjectSlow == true) && (LeMAN_b_AllowedReleaseState == true))
     {
      VeMAN_t_GripperHoldTime = 0;
      if (VeMAN_b_ConeHold == false)
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseCubeSlow;
       }
      else
       {
        /* We are eitehr in cone mode or main intake*/
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseConeSlow;
       }
     }
   else if ((LeMAN_b_DropObjectFast == true) && (LeMAN_b_AllowedReleaseState == true))
     {
      VeMAN_t_GripperHoldTime = 0;
      if (VeMAN_b_ConeHold == false)
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseCubeFast;
       }
      else
       {
        /* We are eitehr in cone mode or main intake*/
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseConeFast;
       }
     }
   else if ((VeMAN_t_GripperHoldTime < KeMAN_t_GripperPullInTm) &&
            (((LeMAN_e_AttndState == E_MAN_MainIntake)    && (LeMAN_e_CmndState  == E_MAN_MainIntake)) ||
             ((LeMAN_e_AttndState == E_MAN_MidCubeIntake) && (LeMAN_e_CmndState  == E_MAN_MidCubeIntake))))
     {
     LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeCube;
     if (VsMAN_s_Sensors.b_GripperObjDetected == true)
       {
        VeMAN_t_GripperHoldTime += C_ExeTime;
       }
       else
       {
        VeMAN_t_GripperHoldTime = 0;
       }
     }
   else if ((VeMAN_t_GripperHoldTime < KeMAN_t_GripperPullInTm) &&
             (((LeMAN_e_AttndState == E_MAN_MidConeIntake)   && (LeMAN_e_CmndState  == E_MAN_MidConeIntake))))
     {
     LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeCone;
      if (VsMAN_s_Sensors.b_GripperObjDetected == true)
       {
        VeMAN_t_GripperHoldTime += C_ExeTime;
       }
      else
       {
        VeMAN_t_GripperHoldTime = 0;
       }
     }
   else if ((VsMAN_s_Sensors.b_GripperObjDetected == true) || (VeMAN_t_GripperHoldTime >= KeMAN_t_GripperPullInTm))
     {
      if (VeMAN_b_ConeHold == true)
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeholdCone;
       }
      else
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeholdCube;
       }
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
                            bool                    LeMAN_b_DropObjectSlow,
                            bool                    LeMAN_b_DropObjectFast)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;
  double LeMAN_Deg_Error = 0.0;
  double LeMAN_k_P_Gain = 0.0;

  if (LeMAN_b_TestPowerOverride == true)
    {
    // Do nothing.  Robot is in test state using power commands for all the acutators
    }
  else if (VeMAN_b_TestState == true)
    {
    /* Only used for testing/calibration. */
     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_ArmPivot] / KeENC_k_ArmPivot, 
                                                             VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot],
                                                             VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot]);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_LinearSlide], 
                                                                VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                                VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide]);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Wrist] / KeENC_Deg_Wrist, 
                                                          VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist],
                                                          VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist]);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Gripper], 
                                                          VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper],
                                                          1.0);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_IntakeRollers] / KeENC_RPM_IntakeRollers, 
                                                                  VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers],
                                                                  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_IntakeRollers]);

     VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm] = VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm];
    }
  else
    {
    /* This is the actual manipulator control */
    VeMAN_b_CriteriaMet = Update_Command_Atained_State(VeMAN_b_CriteriaMet,
                                                       LeMAN_e_SchedState);

    UpdateManipulatorActuators(VeMAN_e_CmndState, VeMAN_e_AttndState);

    UpdateGripperActuator(VeMAN_e_CmndState,
                          VeMAN_e_AttndState,
                          LeMAN_b_DropObjectSlow,
                          LeMAN_b_DropObjectFast);  // Need to come up with object detected

    if ((LeMAN_e_SchedState != VeMAN_e_CmndState) ||
        (LeMAN_e_SchedState != VeMAN_e_AttndState))
      {
        UpdateManipulatorActuators(VeMAN_e_CmndState, VeMAN_e_AttndState);

        VeMAN_b_CriteriaMet = CmndStateReached(VeMAN_e_CmndState);
      }
    }

    /* Final output to the motor command that will be sent to the motor controller: */
    VsMAN_s_Motors.k_MotorCmnd[E_MAN_ArmPivot] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_LinearSlide] =  -Control_PID( VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                                  VsMAN_s_Sensors.In_LinearSlide,
                                                                 &VaMAN_In_LinearSlideError,
                                                                 &VaMAN_k_LinearSlideIntegral,
                                                                  KaMAN_k_LinearSlidePID_Gx[E_P_Gx],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_I_Gx],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_D_Gx],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_P_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_P_Ll],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_I_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_I_Ll],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_D_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_D_Ll],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_Max_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Wrist] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Gripper] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_IntakeRollers] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers];

    VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] = VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm];
  }