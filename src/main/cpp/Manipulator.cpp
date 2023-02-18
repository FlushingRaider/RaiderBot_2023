/*
  Manipulator.cpp

   Created on: Feb 01, 2022
   Author: Lauren and Chloe

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift go brrrrrrrrrrrrrrrrrrr -chloe
 */

#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"

TeMAN_ManipulatorStates VeMAN_e_SchedState = E_Rest; // Where do we want to end up?
TeMAN_ManipulatorStates VeMAN_e_CmndState  = E_Rest; // What is our next/current step?
TeMAN_ManipulatorStates VeMAN_e_AttndState = E_Rest; // What is our current state?

TeMAN_MotorControl VsMAN_s_Motors; // All of the motor commands for the manipulator/intake motors
TsMAN_Sensor       VsMAN_s_Sensors; // All of the sensor values for the manipulator/intake motors

double VeMAN_Cnt_LayoverTimer = 0; // owo, because Chloe
bool   VeMAN_b_CriteriaMet = false;
bool   VeMAN_b_ArmInitialized = false;
bool   VeMAN_b_WaitingForDriverINS = false;  // Instrumentation only, but indication that we are waiting for the driver to press button for next step.
bool   VeMAN_b_Paused = false; //Checks to see if paused (for testing)

#ifdef LiftXY_Test
bool   VeMAN_b_MoterTestTurret = false; // temporary, we don't want to use the manual overrides
double VMAN_PID_Gx[E_PID_SparkMaxCalSz];
#else
bool VeMAN_b_MoterTestTurret = false;
#endif


/******************************************************************************
 * Function:     LiftMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the Arm and intake motors.
 *               - A through F
 ******************************************************************************/
void ManipulatorMotorConfigsInit(rev::SparkMaxPIDController m_ArmPivotPID,
                                 rev::SparkMaxPIDController m_WristPID,
                                 rev::SparkMaxPIDController m_GripperPID,
                                 rev::SparkMaxPIDController m_IntakeRollersPID)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;
  TeMAN_ManipulatorStates  LeMAN_Cnt_Index1;

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
      VsMAN_s_Motors.k_MotorRampRate[LeMAN_i_Index] = KaMAN_k_ManipulatorRate[LeMAN_i_Index];
    }
  
  #ifdef LiftXY_Test
  T_PID_SparkMaxCal LeLFT_e_Index = E_kP;

  for (LeLFT_e_Index = E_kP;
       LeLFT_e_Index < E_PID_SparkMaxCalSz;
       LeLFT_e_Index = T_PID_SparkMaxCal(int(LeLFT_e_Index) + 1))
      {
      V_LiftPID_Gx[LeLFT_e_Index] = K_LiftPID_Gx[LeLFT_e_Index];
      }
  
  // display PID coefficients on SmartDashboard
  // frc::SmartDashboard::PutNumber("P Gain", K_LiftPID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain", K_LiftPID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain", K_LiftPID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone", K_LiftPID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Feed Forward", K_LiftPID_Gx[E_kFF]);
  // frc::SmartDashboard::PutNumber("Max Output", K_LiftPID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output", K_LiftPID_Gx[E_kMinOutput]);

  // display secondary coefficients
  // frc::SmartDashboard::PutNumber("Set Position Y", 0);
  // frc::SmartDashboard::PutNumber("Set Position X", 0);

  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_Rest][VeMAN_Cnt_ManIterationNew]",            K_LiftRampRateXD[E_Rest][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_Swiper][VeMAN_Cnt_ManIterationNew]",     K_LiftRampRateXD[E_Swiper][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_DrivingState][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_DrivingState][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_PositioningState][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_PositioningState][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_Rest][E_LiftIteration2]",            K_LiftRampRateXD[E_Rest][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_TradeOff][E_LiftIteration2]",      K_LiftRampRateXD[E_TradeOff][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_TradeOff][E_LiftIteration2]",   K_LiftRampRateXD[E_TradeOff][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_Swiper][E_LiftIteration2]",     K_LiftRampRateXD[E_Swiper][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_DrivingState][E_LiftIteration2]",   K_LiftRampRateXD[E_DrivingState][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_PositioningState][E_LiftIteration2]",   K_LiftRampRateXD[E_PositioningState][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_DroppingTheLoot][E_LiftIteration2]",      K_LiftRampRateXD[E_DroppingTheLoot][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]);

  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_Rest][VeMAN_Cnt_ManIterationNew]",            K_LiftRampRateYD[E_Rest][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_Swiper][VeMAN_Cnt_ManIterationNew]",     K_LiftRampRateYD[E_Swiper][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_DrivingState][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_DrivingState][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_PositioningState][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_PositioningState][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_Rest][E_LiftIteration2]",            K_LiftRampRateYD[E_Rest][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_TradeOff][E_LiftIteration2]",      K_LiftRampRateYD[E_TradeOff][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_TradeOff][E_LiftIteration2]",   K_LiftRampRateYD[E_TradeOff][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_Swiper][E_LiftIteration2]",     K_LiftRampRateYD[E_Swiper][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_DrivingState][E_LiftIteration2]",   K_LiftRampRateYD[E_DrivingState][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_PositioningState][E_LiftIteration2]",   K_LiftRampRateYD[E_PositioningState][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_DroppingTheLoot][E_LiftIteration2]",      K_LiftRampRateYD[E_DroppingTheLoot][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]", K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]",      K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]",         K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]",        K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]);
  #endif
  }


/******************************************************************************
 * Function:     LiftMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the manipulator motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void LiftMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
                         rev::SparkMaxPIDController m_liftpidXD)
  {
  // read PID coefficients from SmartDashboard
  #ifdef LiftXY_Test
  // double L_p = frc::SmartDashboard::GetNumber("P Gain", 0);
  // double L_i = frc::SmartDashboard::GetNumber("I Gain", 0);
  // double L_d = frc::SmartDashboard::GetNumber("D Gain", 0);
  // double L_iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  // double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  // double L_max = frc::SmartDashboard::GetNumber("Max Output", 0);
  // double L_min = frc::SmartDashboard::GetNumber("Min Output", 0);
  // double L_maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
  // double L_minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
  // double L_maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
  // double L_allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

  // VeMAN_Cnt_MoterTestLocationTurret = frc::SmartDashboard::GetNumber("Set Position Y", 0);
  // VeMAN_Cnt_MoterTestLocationArmPivot = frc::SmartDashboard::GetNumber("Set Position X", 0);

  // if((L_p != V_LiftPID_Gx[E_kP]))   { m_liftpidYD.SetP(L_p); m_liftpidXD.SetP(L_p); V_LiftPID_Gx[E_kP] = L_p; }
  // if((L_i != V_LiftPID_Gx[E_kI]))   { m_liftpidYD.SetI(L_i); m_liftpidXD.SetI(L_i); V_LiftPID_Gx[E_kI] = L_i; }
  // if((L_d != V_LiftPID_Gx[E_kD]))   { m_liftpidYD.SetD(L_d); m_liftpidXD.SetD(L_d); V_LiftPID_Gx[E_kD] = L_d; }
  // if((L_iz != V_LiftPID_Gx[E_kIz])) { m_liftpidYD.SetIZone(L_iz); m_liftpidXD.SetIZone(L_iz); V_LiftPID_Gx[E_kIz] = L_iz; }
  // if((L_ff != V_LiftPID_Gx[E_kFF])) { m_liftpidYD.SetFF(L_ff); m_liftpidXD.SetFF(L_ff); V_LiftPID_Gx[E_kFF] = L_ff; }
  // if((L_max != V_LiftPID_Gx[E_kMaxOutput]) || (L_min != V_LiftPID_Gx[E_kMinOutput])) { m_liftpidYD.SetOutputRange(L_min, L_max); m_liftpidXD.SetOutputRange(L_min, L_max); V_LiftPID_Gx[E_kMinOutput] = L_min; V_LiftPID_Gx[E_kMaxOutput] = L_max; }
  
  VaMAN_InS_RampRateMoterShoulder[E_Rest][VeMAN_Cnt_ManIterationNew]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_Rest][VeMAN_Cnt_ManIterationNew]",            VaMAN_InS_RampRateMoterShoulder[E_Rest][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_TradeOff][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterShoulder[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_TradeOff][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterShoulder[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_Swiper][VeMAN_Cnt_ManIterationNew]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_Swiper][VeMAN_Cnt_ManIterationNew]",     VaMAN_InS_RampRateMoterShoulder[E_Swiper][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_DrivingState][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_DrivingState][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterShoulder[E_DrivingState][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_PositioningState][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_PositioningState][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterShoulder[E_PositioningState][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterShoulder[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", VaMAN_InS_RampRateMoterShoulder[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterShoulder[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         VaMAN_InS_RampRateMoterShoulder[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        VaMAN_InS_RampRateMoterShoulder[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterShoulder[E_Rest][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_Rest][E_LiftIteration2]",            VaMAN_InS_RampRateMoterShoulder[E_Rest][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_TradeOff][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_TradeOff][E_LiftIteration2]",      VaMAN_InS_RampRateMoterShoulder[E_TradeOff][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_TradeOff][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_TradeOff][E_LiftIteration2]",   VaMAN_InS_RampRateMoterShoulder[E_TradeOff][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_Swiper][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_Swiper][E_LiftIteration2]",     VaMAN_InS_RampRateMoterShoulder[E_Swiper][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_DrivingState][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_DrivingState][E_LiftIteration2]",   VaMAN_InS_RampRateMoterShoulder[E_DrivingState][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_PositioningState][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_PositioningState][E_LiftIteration2]",   VaMAN_InS_RampRateMoterShoulder[E_PositioningState][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_DroppingTheLoot][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_DroppingTheLoot][E_LiftIteration2]",      VaMAN_InS_RampRateMoterShoulder[E_DroppingTheLoot][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", VaMAN_InS_RampRateMoterShoulder[E_S8_more_down_some_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterShoulder[E_S9_back_rest_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         VaMAN_InS_RampRateMoterShoulder[E_S10_final_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterShoulder[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        VaMAN_InS_RampRateMoterShoulder[E_S11_final_OWO][E_LiftIteration2]);

  VaMAN_InS_RampRateMoterTurret[E_Rest][VeMAN_Cnt_ManIterationNew]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_Rest][VeMAN_Cnt_ManIterationNew]",            VaMAN_InS_RampRateMoterTurret[E_Rest][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_TradeOff][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterTurret[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_TradeOff][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_TradeOff][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterTurret[E_TradeOff][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_Swiper][VeMAN_Cnt_ManIterationNew]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_Swiper][VeMAN_Cnt_ManIterationNew]",     VaMAN_InS_RampRateMoterTurret[E_Swiper][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_DrivingState][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_DrivingState][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterTurret[E_DrivingState][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_PositioningState][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_PositioningState][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterTurret[E_PositioningState][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterTurret[E_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", VaMAN_InS_RampRateMoterTurret[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterTurret[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         VaMAN_InS_RampRateMoterTurret[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        VaMAN_InS_RampRateMoterTurret[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterTurret[E_Rest][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_Rest][E_LiftIteration2]",            VaMAN_InS_RampRateMoterTurret[E_Rest][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_TradeOff][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_TradeOff][E_LiftIteration2]",      VaMAN_InS_RampRateMoterTurret[E_TradeOff][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_TradeOff][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_TradeOff][E_LiftIteration2]",   VaMAN_InS_RampRateMoterTurret[E_TradeOff][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_Swiper][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_Swiper][E_LiftIteration2]",     VaMAN_InS_RampRateMoterTurret[E_Swiper][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_DrivingState][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_DrivingState][E_LiftIteration2]",   VaMAN_InS_RampRateMoterTurret[E_DrivingState][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_PositioningState][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_PositioningState][E_LiftIteration2]",   VaMAN_InS_RampRateMoterTurret[E_PositioningState][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_DroppingTheLoot][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_DroppingTheLoot][E_LiftIteration2]",      VaMAN_InS_RampRateMoterTurret[E_DroppingTheLoot][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]", VaMAN_InS_RampRateMoterTurret[E_S8_more_down_some_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterTurret[E_S9_back_rest_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]",         VaMAN_InS_RampRateMoterTurret[E_S10_final_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterTurret[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]",        VaMAN_InS_RampRateMoterTurret[E_S11_final_OWO][E_LiftIteration2]);
  #endif
  }

/******************************************************************************
 * Function:     LiftControlInit
 *
 * Description:  Initialization function for the lift control.
 ******************************************************************************/
void LiftControlInit()
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;

  VeMAN_e_SchedState = E_Rest;
  VeMAN_e_CmndState  = E_Rest;
  VeMAN_e_AttndState = E_Rest;

  VeMAN_Cnt_LayoverTimer = 0;
  VeMAN_b_CriteriaMet = false;
  VeMAN_b_Paused = false;
  VeMAN_b_WaitingForDriverINS = false;
  VeMAN_b_ArmInitialized = false;

  for (LeMAN_i_Index = E_MAN_Turret;
       LeMAN_i_Index < E_MAN_Sz;
       LeMAN_i_Index = TeMAN_e_ManipulatorActuator(int(LeMAN_i_Index) + 1))
    {
      VsMAN_s_Motors.k_MotorCmnd[LeMAN_i_Index] = 0.0;
      VsMAN_s_Motors.k_MotorRampRate[LeMAN_i_Index] = KaMAN_k_ManipulatorRate[LeMAN_i_Index];
      VsMAN_s_Motors.k_MotorTestPower[LeMAN_i_Index] = 0.0;
      VsMAN_s_Motors.k_MotorTestValue[LeMAN_i_Index] = 0.0;
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
      VsMAN_s_Motors.k_MotorRampRate[LeMAN_i_Index] = KaMAN_k_ManipulatorRate[LeMAN_i_Index];
      VsMAN_s_Motors.k_MotorTestPower[LeMAN_i_Index] = 0.0;
    }

  if (LsCONT_s_DriverInput->b_IntakeArmIn == true)
    {
    VsMAN_s_Motors.k_MotorTestPower[E_MAN_IntakeArm] = KaMAN_k_ManipulatorTestPower[E_MAN_IntakeArm];
    }

  if (LsCONT_s_DriverInput->b_IntakeArmOut == true)
    {
    VsMAN_s_Motors.k_MotorTestPower[E_MAN_IntakeArm] = -KaMAN_k_ManipulatorTestPower[E_MAN_IntakeArm];
    }

  if (LsCONT_s_DriverInput->b_IntakeRollers == true)
    {
    VsMAN_s_Motors.k_MotorTestPower[E_MAN_IntakeRollers] = KaMAN_k_ManipulatorTestPower[E_MAN_IntakeRollers];
    }

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_ArmPivot] = LsCONT_s_DriverInput->pct_ArmPivot * KaMAN_k_ManipulatorTestPower[E_MAN_ArmPivot];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_LinearSlide] = LsCONT_s_DriverInput->pct_ArmPivot * KaMAN_k_ManipulatorTestPower[E_MAN_LinearSlide];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_Turret] = LsCONT_s_DriverInput->pct_Turret * KaMAN_k_ManipulatorTestPower[E_MAN_Turret];

  VsMAN_s_Motors.k_MotorTestPower[E_MAN_Gripper] = LsCONT_s_DriverInput->pct_Claw * KaMAN_k_ManipulatorTestPower[E_MAN_Gripper];
  }




 
#ifdef InProcess
/******************************************************************************
 * Function:     Rest_State
 *
 * Description:  Everything in default positions.
 ******************************************************************************/
 bool Rest_State()
  {
  bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandTurret = K_lift_S2_YD;

  *LeMAN_Cmd_CommandArmPivot = K_lift_min_XD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_TradeOff][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_TradeOff][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S2_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S2_YD - K_lift_deadband_YD)) {
    LeLFT_b_CriteriaMet = true;
  }

  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:     Intake_State,
 *
 * Description:  State 2: robert intake or somthin
 ******************************************************************************/
 bool       Intake_State()
{
  bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandTurret = K_lift_S2_YD;

  *LeMAN_Cmd_CommandArmPivot = K_lift_min_XD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_TradeOff][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_TradeOff][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S2_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S2_YD - K_lift_deadband_YD)) {
    LeLFT_b_CriteriaMet = true;
  }

  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:     TradeOff_State,
 *
 * Description:  State 2: moving ball from intake to manipulator?
 ******************************************************************************/
 bool     TradeOff_State()
{
  bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandTurret = K_lift_S2_YD;

  *LeMAN_Cmd_CommandArmPivot = K_lift_min_XD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_TradeOff][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_TradeOff][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S2_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S2_YD - K_lift_deadband_YD)) {
    LeLFT_b_CriteriaMet = true;
  }

  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:      Swipe_State,
 *
 * Description:  State 3: swiper no swiping
 ******************************************************************************/
 bool          Swipe_State()  
{
  bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandArmPivot = K_lift_S3_XD;

  *LeMAN_Cmd_CommandTurret = K_lift_S3_YD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_TradeOff][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_TradeOff][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleArmPivot <= (K_lift_S3_XD + K_lift_deadband_XD) && LeMAN_Deg_MeasuredAngleArmPivot >= (K_lift_S3_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeLFT_Cnt_LiftDebounceTimer = 0;
    }
  }
  else {
    VeLFT_Cnt_LiftDebounceTimer = 0;
  }

  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:       Driving_State,
 *
 * Description:  State 4: x lift no move, y lift go
 ******************************************************************************/
 bool   Driving_State()  
{
   bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandTurret = K_lift_S4_YD;

  *LeMAN_Cmd_CommandArmPivot = K_lift_S4_XD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_Swiper][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_Swiper][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S4_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S4_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeMAN_b_AutoManipulateButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeLFT_Cnt_LiftDebounceTimer = 0;
         VeLFT_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeLFT_Cnt_LiftDebounceTimer = 0;
  }
  
  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:       Out_State,
 *
 * Description:  State 5: y lift no move, x lift go
 ******************************************************************************/
 bool   Positioning_State()  
{
  bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandArmPivot = K_lift_S5_XD;

  *LeMAN_Cmd_CommandTurret = K_lift_S5_YD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_DrivingState][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_DrivingState][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleArmPivot <= (K_lift_S5_XD + K_lift_deadband_XD) && LeMAN_Deg_MeasuredAngleArmPivot >= (K_lift_S5_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeLFT_Cnt_LiftDebounceTimer = 0;
    }
  }
  else {
    VeLFT_Cnt_LiftDebounceTimer = 0;
  }
  
  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:       DroppingTheLoot_State,
 *
 * Description:  State 6: y lift go down, x lift bad stop what's in your mouth no get back here doN'T EAT IT
 ******************************************************************************/
 bool DroppingTheLoot_State()  
{
  bool LeLFT_b_CriteriaMet = false;
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Turret} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_ArmPivot} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_LinearSlide} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Wrist} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Claw} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);
 VsMAN_s_Motors.k_MotorCmnd{E_MAN_Intake} = RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandTurret = K_lift_S6_YD;

  *LeMAN_Cmd_CommandArmPivot = K_lift_S6_XD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_DroppingTheLoot][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_DroppingTheLoot][LeMAN_CmdStateIteration];

  if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S6_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S6_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeLFT_Cnt_LiftDebounceTimer = 0;
    }
  }
  else {
    VeLFT_Cnt_LiftDebounceTimer = 0;
  }
  
  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:       S7_move_back_XD,
 *
 * Description:  State 7: X go back-aroni, we look at gyro to make sure we aren't tilted too much
 ******************************************************************************/
//  bool S7_move_back_XD(double         LeMAN_b_AutoManipulateButton,
//                       double         LeMAN_Deg_MeasuredAngleTurret,
//                       double         LeMAN_Deg_MeasuredAngleArmPivot,
//                       double         LeMAN_In_MeasuredPositionLinearSlide,
//                       double         LeMAN_Deg_MeasuredAngleWrist,
//                       double         LeMAN_Deg_MeasuredAngleClaw,
//                       double         LeMAN_RPM_MeasuredSpeedIntake,
//                       double         LeLEFT_Deg_GyroAngleYaws,
//                       double        *LeMAN_Cmd_CommandTurret,
//                       double        *LeMAN_Cmd_CommandArmPivot,
//                       double        *LeMAN_Cmd_MeasuredPositionLinearSlide,
//                       double        *LeMAN_Cmd_MeasuredAngleWrist,
//                       double        *LeMAN_Cmd_MeasuredAngleClaw,
//                       double        *LeMAN_Cmd_MeasuredSpeedIntake,
//                       double        *LeMAN_InS_CommandRateTurret,
//                       double        *LeMAN_InS_CommandRateArmPivot,
//                       T_Man_Iteration LeMAN_CmdStateIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeMAN_Cmd_CommandArmPivot = K_lift_S7_XD;

  *LeMAN_Cmd_CommandTurret = K_lift_S7_YD;

  *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_DroppingTheLoot][LeMAN_CmdStateIteration];

  *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_DroppingTheLoot][LeMAN_CmdStateIteration]; // Don't go too fast, going slower will help to reduce rocking

  if (LeMAN_Deg_MeasuredAngleArmPivot <= (K_lift_S7_XD + K_lift_deadband_XD)  && LeMAN_Deg_MeasuredAngleArmPivot >= (K_lift_S7_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeMAN_b_AutoManipulateButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeLFT_Cnt_LiftDebounceTimer = 0;
         VeLFT_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeLFT_Cnt_LiftDebounceTimer = 0;
  }
  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:       S8_more_down_some_YD,
 *
 * Description:  State 8: me when the lift go down more
 ******************************************************************************/
//  bool S8_more_down_some_YD(double         LeMAN_b_AutoManipulateButton,
//                            double         LeMAN_Deg_MeasuredAngleTurret,
//                            double         LeMAN_Deg_MeasuredAngleArmPivot,
//                            double         LeMAN_In_MeasuredPositionLinearSlide,
//                            double              LeMAN_Deg_MeasuredAngleWrist,
//                            double         LeMAN_Deg_MeasuredAngleClaw,
//                             double              LeMAN_RPM_MeasuredSpeedIntake,
//                            double        *LeMAN_Cmd_CommandTurret,
//                            double        *LeMAN_Cmd_CommandArmPivot,
//                            double        *LeMAN_InS_CommandRateTurret,
//                            double        *LeMAN_InS_CommandRateArmPivot,
//                            T_Man_Iteration LeMAN_CmdStateIteration)  
// {
//   bool LeLFT_b_CriteriaMet = false;
  
//   *LeMAN_Cmd_CommandTurret = K_lift_S8_YD;

//   *LeMAN_Cmd_CommandArmPivot = K_lift_S8_XD;

//   *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S8_more_down_some_YD][LeMAN_CmdStateIteration];

//   *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S8_more_down_some_YD][LeMAN_CmdStateIteration];

//   if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S8_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S8_YD - K_lift_deadband_YD)) {
//     VeMAN_Cnt_LayoverTimer += C_ExeTime;
//     if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
//       VeMAN_b_WaitingForDriverINS = true;
//       if (LeMAN_b_AutoManipulateButton == true){
//          /* Let the driver determine when we are not swinging and can proceed */
//          LeLFT_b_CriteriaMet = true;
//          VeMAN_Cnt_LayoverTimer = 0;
//          VeMAN_b_WaitingForDriverINS = false;
//       }
//     }
//   }
//   else {
//     VeMAN_Cnt_LayoverTimer = 0;
//   }
  
//   return(LeLFT_b_CriteriaMet);
// }

/******************************************************************************
 * Function:       S9_back_rest_XD
 *
 * Description:  State 9: reset it to initial x position (we aren't fixing my back  :(  )
 ******************************************************************************/
//  bool S9_back_rest_XD(double         LeMAN_b_AutoManipulateButton,
//                       double         LeMAN_Deg_MeasuredAngleTurret,
//                       double         LeMAN_Deg_MeasuredAngleArmPivot,
//                       double         LeMAN_In_MeasuredPositionLinearSlide,
//                        double              LeMAN_Deg_MeasuredAngleWrist,
//                        double         LeMAN_Deg_MeasuredAngleClaw,
//                        double              LeMAN_RPM_MeasuredSpeedIntake,
//                       double        *LeMAN_Cmd_CommandTurret,
//                       double        *LeMAN_Cmd_CommandArmPivot,
//                       double        *LeMAN_InS_CommandRateTurret,
//                       double        *LeMAN_InS_CommandRateArmPivot,
//                       T_Man_Iteration LeMAN_CmdStateIteration)  
// {
//   bool LeLFT_b_CriteriaMet = false;
  
//   *LeMAN_Cmd_CommandArmPivot = K_lift_S9_XD;

//   *LeMAN_Cmd_CommandTurret = K_lift_S9_YD;

//   *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S9_back_rest_XD][LeMAN_CmdStateIteration];

//   *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S9_back_rest_XD][LeMAN_CmdStateIteration];

//   if (LeMAN_Deg_MeasuredAngleArmPivot <= (K_lift_S9_XD + K_lift_deadband_XD) && LeMAN_Deg_MeasuredAngleArmPivot >= (K_lift_S9_XD - K_lift_deadband_XD)) {
//     VeMAN_Cnt_LayoverTimer += C_ExeTime;
//     if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
//       VeMAN_b_WaitingForDriverINS = true;
//       if (LeMAN_b_AutoManipulateButton == true){
//          /* Let the driver determine when we are not swinging and can proceed */
//          LeLFT_b_CriteriaMet = true;
//          VeMAN_Cnt_LayoverTimer = 0;
//          VeMAN_b_WaitingForDriverINS = false;
//       }
//     }
//   }
//   else {
//     VeMAN_Cnt_LayoverTimer = 0;
//   }
  
//   return(LeLFT_b_CriteriaMet);
// }

/******************************************************************************
 * Function:       S10_final_YD
 *
 * Description:  State 10: y move down, robert move up (what a chad)
 ******************************************************************************/
//  bool S10_final_YD(double         LeMAN_b_AutoManipulateButton,
//                    double         LeMAN_Deg_MeasuredAngleTurret,
//                    double         LeMAN_Deg_MeasuredAngleArmPivot,
//                    double         LeMAN_In_MeasuredPositionLinearSlide,
//                    double              LeMAN_Deg_MeasuredAngleWrist,
//                    double         LeMAN_Deg_MeasuredAngleClaw,
//                    double              LeMAN_RPM_MeasuredSpeedIntake,
//                    double        *LeMAN_Cmd_CommandTurret,
//                    double        *LeMAN_Cmd_CommandArmPivot,
//                    double        *LeMAN_InS_CommandRateTurret,
//                    double        *LeMAN_InS_CommandRateArmPivot,
//                    T_Man_Iteration LeMAN_CmdStateIteration)  
// {
//   bool LeLFT_b_CriteriaMet = false;
  
//   *LeMAN_Cmd_CommandTurret = K_lift_S10_YD;

//   *LeMAN_Cmd_CommandArmPivot = K_lift_S10_XD;

//   *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S10_final_YD][LeMAN_CmdStateIteration]; // Slow down, don't yank too hard

//   *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S10_final_YD][LeMAN_CmdStateIteration];

//   if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S10_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S10_YD - K_lift_deadband_YD)) {
//     VeMAN_Cnt_LayoverTimer += C_ExeTime;
//     if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
//          LeLFT_b_CriteriaMet = true;
//          VeMAN_Cnt_LayoverTimer = 0;
//     }
//   }
//   else {
//     VeMAN_Cnt_LayoverTimer = 0;
//   }
  
//   return(LeLFT_b_CriteriaMet);
// }

/******************************************************************************
 * Function:       S11_final_OWO
 *
 * Description:  State 11: uwu
 ******************************************************************************/
//  bool S11_final_OWO(double         LeMAN_b_AutoManipulateButton,
//                     double         LeMAN_Deg_MeasuredAngleTurret,
//                     double         LeMAN_Deg_MeasuredAngleArmPivot,
//                     double         LeMAN_In_MeasuredPositionLinearSlide,
//                     double              LeMAN_Deg_MeasuredAngleWrist,
//                     double         LeMAN_Deg_MeasuredAngleClaw,
//                     double              LeMAN_RPM_MeasuredSpeedIntake,
//                     double        *LeMAN_Cmd_CommandTurret,
//                     double        *LeMAN_Cmd_CommandArmPivot,
//                     double        *LeMAN_InS_CommandRateTurret,
//                     double        *LeMAN_InS_CommandRateArmPivot,
//                     T_Man_Iteration LeMAN_CmdStateIteration)  
// {
//   bool LeLFT_b_CriteriaMet = false;
  
//   *LeMAN_Cmd_CommandTurret = K_lift_S11_YD;

//   *LeMAN_Cmd_CommandArmPivot = K_lift_S11_XD;

//   *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S11_final_OWO][LeMAN_CmdStateIteration];

//   *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S11_final_OWO][LeMAN_CmdStateIteration];

//   if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S11_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S11_YD - K_lift_deadband_YD)) {
//     VeMAN_Cnt_LayoverTimer += C_ExeTime;
//     if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
//       VeMAN_b_WaitingForDriverINS = true;
//       if (LeMAN_b_AutoManipulateButton == true){
//          /* Let the driver determine when we are not swinging and can proceed */
//          LeLFT_b_CriteriaMet = true;
//          VeMAN_Cnt_LayoverTimer = 0;
//          VeMAN_b_WaitingForDriverINS = false;
//       }
//     }
//   }
//   else {
//     VeMAN_Cnt_LayoverTimer = 0;
//   }
  
//   return(LeLFT_b_CriteriaMet);
// }


/******************************************************************************
 * Function:     ManipulatorControlDictator
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/
TeMAN_ManipulatorStates ManipulatorControlDictator(bool                LeMAN_b_AutoManipulateButton,
                                   bool                LeLFT_b_DriverAutoClimbPause,
                                   TeLFT_e_LiftCmndDirection LeLFT_Cmd_DriverLiftDirection,
                                   double              LeLFT_SEC_GameTime,
                                   TeMAN_ManipulatorStates        LeMAN_Cnt_CurrentState,                                
                                   double              LeMAN_Deg_MeasuredAngleTurret,
                                   double              LeMAN_Deg_MeasuredAngleArmPivot,
                                   double              LeMAN_In_MeasuredPositionLinearSlide,
                                   double              LeMAN_Deg_MeasuredAngleWrist,
                                   double              LeMAN_Deg_MeasuredAngleClaw,
                                   double              LeMAN_RPM_MeasuredSpeedIntake,
                                   double             *LeMAN_Cmd_CommandTurret,
                                   double             *LeMAN_Cmd_CommandArmPivot,
                                   double             *LeLFT_Pct_CommandPwrYD,
                                   double             *LeLFT_Pct_CommandPwrXD,
                                   bool                LeMAN_b_LimitDetectedTurret,
                                   bool                LeMAN_b_LimitDetectedArmPivot,
                                   double              LeLEFT_Deg_GyroAngleYaws,
                                   double              LeMAN_v_MotorCurrentOutTurret,
                                   double              LeMAN_v_MotorCurrentOutArmPivot,
                                   double              LeMAN_v_MotorCurrentOutLinearSlide,
                                   double              LeMAN_v_MotorCurrentOutWrist,
                                   double              LeMAN_v_MotorCurrentOutClaw,
                                   double              LeMAN_v_MotorCurrentOutIntake,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD)
  {
  TeMAN_ManipulatorStates LeLFT_e_CommandedState = LeMAN_Cnt_CurrentState;
  double LeMAN_Cmd_CommandTurret_Temp = 0;
  double LeMAN_Cmd_CommandArmPivot_Temp = 0;
  double LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_Rest][VeMAN_Cnt_ManIterationNew];
  double LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_Rest][VeMAN_Cnt_ManIterationNew];
  double LeMAN_v_MoterPowerTurret= 0;
  double LeMAN_v_MoterPowerArmPivot= 0;

  if (VeMAN_b_MoterTestTurret == true)
    {
    /* Only used for testing. */
    LeMAN_Cmd_CommandTurret_Temp = VeMAN_Cnt_MoterTestLocationTurret;
    LeMAN_Cmd_CommandArmPivot_Temp = VeMAN_Cnt_MoterTestLocationArmPivot;
    }
  else if (VeLFT_b_LiftInitialized == false)
    {
    if (LeMAN_b_LimitDetectedTurret == false)
      {
      LeMAN_v_MoterPowerTurret= K_lift_autoResetDown_YD;
      }

    if (LeMAN_b_LimitDetectedArmPivot == false)
      {
      LeMAN_v_MoterPowerArmPivot= K_lift_driver_manual_back_XD;
      }
    
    if (LeMAN_b_LimitDetectedTurret == true && 
        LeMAN_b_LimitDetectedArmPivot == true)
      {
      LeMAN_v_MoterPowerTurret= 0;
      LeMAN_v_MoterPowerArmPivot= 0;
      VeMAN_b_ArmInitialized = true;

      // EncodersLiftInit(m_encoderLiftYD,
      //                  m_encoderLiftXD);
      }
    }
  else if ((LeLFT_b_DriverAutoClimbPause == true) && (VeLFT_b_Paused == false))
    {
    /* The driver pressed a button to puase the climb process.  Let's save the current locations and hold. */
    VeMAN_b_Paused = true;
    VeMAN_Cnt_MoterTestPowerCmndTurret = LeMAN_Deg_MeasuredAngleArmPivot;
    VeMAN_Cnt_MoterTestPowerCmndArmPivot = LeMAN_Deg_MeasuredAngleTurret;
    /* Set commanded location to current measured location for this loop. */
    LeMAN_Cmd_CommandArmPivot_Temp = LeMAN_Deg_MeasuredAngleArmPivot;
    LeMAN_Cmd_CommandTurret_Temp = LeMAN_Deg_MeasuredAngleTurret;
    }
  else if (((LeMAN_b_AutoManipulateButton == true) && (VeMAN_b_Paused == true)) || 
            (VeMAN_b_Paused == false))
    {
    VeMAN_b_Paused = false;
    VeMAN_Cnt_MoterTestPowerCmndTurret = LeMAN_Deg_MeasuredAngleArmPivot;
    VeMAN_Cnt_MoterTestPowerCmndArmPivot = LeMAN_Deg_MeasuredAngleTurret;

    switch (LeLFT_Cnt_CurrentState)
      {
        case :
            if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndUp)
              {
              LeMAN_Cmd_CommandTurret_Temp = *LeMAN_Cmd_CommandTurret + K_lift_driver_up_rate_YD;
              }
            else if (LeLFT_Cmd_DriverLiftDirection == E_LiftCmndDown)
              {
              LeMAN_Cmd_CommandTurret_Temp = *LeMAN_Cmd_CommandTurret - K_lift_driver_down_rate_YD;
              }
              else 
              {
                LeMAN_Cmd_CommandTurret_Temp = *LeMAN_Cmd_CommandTurret;
              }
            /* The driver should only initiate the state machine once the robot has become suspended. */
            if (LeMAN_b_AutoManipulateButton == true && LeMAN_Deg_MeasuredAngleTurret >= K_lift_enable_auto_YD) {
                LeLFT_e_CommandedState = E_Rest;
            }
        break;

        case E_Rest:
            VeMAN_b_CriteriaMet = S0_Rest(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_TradeOff;
            }
        break;

        case E_TradeOff:
            VeMAN_b_CriteriaMet = S2_TradeOff(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_Swiper;
            }
        break;

        case E_Swiper:
            VeMAN_b_CriteriaMet = S3_Swiper(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_DrivingState;
            }
        break;

        case E_DrivingState:
            VeMAN_b_CriteriaMet = S4_DrivingState(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_PositioningState;
            }
        break;

        case E_PositioningState:
            VeMAN_b_CriteriaMet = S5_Positioning(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_DroppingTheLoot;
            }
        break;

        case E_DroppingTheLoot:
            VeMAN_b_CriteriaMet = S6_DroppingTheLoot(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot, VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_Rest;
            }
        break;

        // case E_S8_more_down_some_YD:
        //     VeMAN_b_CriteriaMet = S8_more_down_some_YD(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
        //     if(VeMAN_b_CriteriaMet == true){
        //       LeLFT_e_CommandedState =   E_S9_back_rest_XD;
        //     }
        // break;

        // case E_S9_back_rest_XD:
        //     VeMAN_b_CriteriaMet = S9_back_rest_XD(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
        //     if(VeMAN_b_CriteriaMet == true){
        //       LeLFT_e_CommandedState =   E_S10_final_YD;
        //     }
        // break;

        // case E_S10_final_YD:
        //     VeMAN_b_CriteriaMet = S10_final_YD(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
        //     if(VeMAN_b_CriteriaMet == true){
        //       LeLFT_e_CommandedState = E_S11_final_OWO;
        //     }
        // break;

        // case E_S11_final_OWO:
        //     VeMAN_b_CriteriaMet = S11_final_OWO(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeMAN_Cnt_ManIteration);
        //     if(VeMAN_b_CriteriaMet == true &&VeMAN_Cnt_ManIteration < E_LiftIteration2){
        //       LeLFT_e_CommandedState = E_TradeOff;
        //      VeMAN_Cnt_ManIteration = E_LiftIteration2;
        //     }
        //     else if(VeMAN_b_CriteriaMet == true &&VeMAN_Cnt_ManIteration >= E_LiftIteration2){
        //       LeLFT_e_CommandedState = E_S11_final_OWO;
        //     }
        // break;
      }
    }
  else
    {
    /* Lift is currently paused: */
    LeMAN_Cmd_CommandArmPivot_Temp = VeMAN_Cnt_MoterTestPowerCmndTurret;
    LeMAN_Cmd_CommandTurret_Temp = VeMAN_Cnt_MoterTestPowerCmndArmPivot;
    }

  /* Place limits on the travel of XD and YD to prevent damage: */
  if (LeMAN_Cmd_CommandTurret_Temp > K_lift_max_YD)
    {
    LeMAN_Cmd_CommandTurret_Temp = K_lift_max_YD;
    }
  else if (LeMAN_Cmd_CommandTurret_Temp < K_lift_min_YD)
    {
    LeMAN_Cmd_CommandTurret_Temp = K_lift_max_YD;
    }

  if (LeMAN_Cmd_CommandArmPivot_Temp > K_lift_max_XD)
    {
    LeMAN_Cmd_CommandArmPivot_Temp = K_lift_max_XD;
    }
  else if (LeMAN_Cmd_CommandArmPivot_Temp < K_lift_min_XD)
    {
    LeMAN_Cmd_CommandArmPivot_Temp = K_lift_max_XD;
    }

  *LeMAN_Cmd_CommandTurret= RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

  *LeMAN_Cmd_CommandArmPivot= RampTo(LeMAN_Cmd_CommandArmPivot_Temp, *LeMAN_Cmd_CommandArmPivot, LeMAN_InS_CommandRateArmPivot);

  *LeLFT_Pct_CommandPwrYD = LeMAN_v_MoterPowerTurret;
  
  *LeLFT_Pct_CommandPwrXD = LeMAN_v_MoterPowerArmPivot;

  return(LeLFT_e_CommandedState);
}
#endif