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
#ifdef BrokenMain
T_Man_DoesStuffMaybe VeMAN_CnT_Man_DoesStuffMaybe = E_S0_Rest; 
T_Man_Iteration VeMAN_Cnt_ManIteration = VeMAN_Cnt_ManIterationNew;
T_Lift_State VeLFT_Cnt_Lift_state = E_S0_BEGONE; 
T_Lift_Iteration VeLFT_Cnt_LiftIteration = VeLFT_Cnt_LiftIteration1;

double VeLFT_Cnt_LiftDebounceTimer = 0; // owo, because Chloe
bool   VeLFT_b_CriteriaMet = false;
bool   VeLFT_b_LiftInitialized = false;

double VeLFT_Cnt_CommandYD = 0;
double VeLFT_Cnt_CommandXD = 0;

double VeLFT_Cnt_LiftYDTestLocation = 0;
double VeLFT_Cnt_LiftXDTestLocation = 0;

double VeLFT_Cnt_LiftYDTestPowerCmnd = 0;
double VeLFT_Cnt_LiftXDTestPowerCmnd = 0;

double VaMAN_v_MotorMaxCurrentA[E_Man_State_Sz]; //max current(temp)
double VaMAN_v_MotorMaxCurrentB[E_Man_State_Sz]; //max current(temp)

bool   VeLFT_b_WaitingForDriverINS = false;  // Instrumentation only, but indication that we are waiting for the driver to press button for next step.
bool   VeLFT_b_Paused = false;
double VeLFT_b_PausedXDPosition = 0;
double VeLFT_b_PausedYDPosition = 0;

double VaMAN_InS_RampRateMoterA[E_Man_State_Sz][E_LiftIterationSz]; //motor ramp rate (temp)
double VaMAN_InS_RampRateMoterB[E_Man_State_Sz][E_LiftIterationSz]; //motor ramp rate (temp)

#ifdef LiftXY_Test
bool   VeLFT_b_LiftXYTest = false; // temporary, we don't want to use the manual overrides
double V_LiftPID_Gx[E_PID_SparkMaxCalSz];
#else
bool VeLFT_b_LiftXYTest = false;
#endif


/******************************************************************************
 * Function:     LiftMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the lift motors.
 *               - XD and YD
 ******************************************************************************/
void LiftMotorConfigsInit(rev::SparkMaxPIDController m_liftpidYD,
                          rev::SparkMaxPIDController m_liftpidXD)
  {
  T_Man_DoesStuffMaybe     LeMAN_Cnt_Index1;
  T_Man_Iteration LeMAN_Cnt_Index2;

  // set PID coefficients
  m_liftpidYD.SetP(K_LiftPID_Gx[E_kP]);
  m_liftpidYD.SetI(K_LiftPID_Gx[E_kI]);
  m_liftpidYD.SetD(K_LiftPID_Gx[E_kD]);
  m_liftpidYD.SetIZone(K_LiftPID_Gx[E_kIz]);
  m_liftpidYD.SetFF(K_LiftPID_Gx[E_kFF]);
  m_liftpidYD.SetOutputRange(K_LiftPID_Gx[E_kMinOutput], K_LiftPID_Gx[E_kMaxOutput]);

  m_liftpidXD.SetP(K_LiftPID_Gx[E_kP]);
  m_liftpidXD.SetI(K_LiftPID_Gx[E_kI]);
  m_liftpidXD.SetD(K_LiftPID_Gx[E_kD]);
  m_liftpidXD.SetIZone(K_LiftPID_Gx[E_kIz]);
  m_liftpidXD.SetFF(K_LiftPID_Gx[E_kFF]);
  m_liftpidXD.SetOutputRange(K_LiftPID_Gx[E_kMinOutput], K_LiftPID_Gx[E_kMaxOutput]);

  for (LeLFT_Cnt_Index2 = VeLFT_Cnt_LiftIteration1;
       LeLFT_Cnt_Index2 < E_LiftIterationSz;
       LeLFT_Cnt_Index2 = T_Lift_Iteration(int(LeLFT_Cnt_Index2) + 1))
      {
      for (LeMAN_Cnt_Index1 = E_S0_Rest;
           LeMAN_Cnt_Index1 < E_Man_State_Sz;
           LeMAN_Cnt_Index1 = T_Man_DoesStuffMaybe(int(LeMAN_Cnt_Index1) + 1))
          {
          VaLFT_InS_LiftRampRateYD[LeLFT_Cnt_Index1][LeLFT_Cnt_Index2] = K_LiftRampRateYD[LeLFT_Cnt_Index1][LeLFT_Cnt_Index2];
          VaLFT_InS_LiftRampRateXD[LeLFT_Cnt_Index1][LeLFT_Cnt_Index2] = K_LiftRampRateXD[LeLFT_Cnt_Index1][LeLFT_Cnt_Index2];
          }
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

  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S0_Rest][VeMAN_Cnt_ManIterationNew]",            K_LiftRampRateXD[E_S0_Rest][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S1_Intake][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S1_Intake][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]",     K_LiftRampRateXD[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S0_Rest][E_LiftIteration2]",            K_LiftRampRateXD[E_S0_Rest][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S1_Intake][E_LiftIteration2]",      K_LiftRampRateXD[E_S1_Intake][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S2_TradeOff][E_LiftIteration2]",   K_LiftRampRateXD[E_S2_TradeOff][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S3_Swiper][E_LiftIteration2]",     K_LiftRampRateXD[E_S3_Swiper][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S4_DrivingState][E_LiftIteration2]",   K_LiftRampRateXD[E_S4_DrivingState][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S5_Positioning][E_LiftIteration2]",   K_LiftRampRateXD[E_S5_Positioning][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S6_DroppingTheLoot][E_LiftIteration2]",      K_LiftRampRateXD[E_S6_DroppingTheLoot][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]);

  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S0_Rest][VeMAN_Cnt_ManIterationNew]",            K_LiftRampRateYD[E_S0_Rest][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S1_Intake][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S1_Intake][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]",     K_LiftRampRateYD[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S0_Rest][E_LiftIteration2]",            K_LiftRampRateYD[E_S0_Rest][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S1_Intake][E_LiftIteration2]",      K_LiftRampRateYD[E_S1_Intake][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S2_TradeOff][E_LiftIteration2]",   K_LiftRampRateYD[E_S2_TradeOff][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S3_Swiper][E_LiftIteration2]",     K_LiftRampRateYD[E_S3_Swiper][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S4_DrivingState][E_LiftIteration2]",   K_LiftRampRateYD[E_S4_DrivingState][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S5_Positioning][E_LiftIteration2]",   K_LiftRampRateYD[E_S5_Positioning][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S6_DroppingTheLoot][E_LiftIteration2]",      K_LiftRampRateYD[E_S6_DroppingTheLoot][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]", K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]",      K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]",         K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]",        K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]);
  #endif
  }


/******************************************************************************
 * Function:     LiftMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the lift motors.  This 
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

  // VeLFT_Cnt_LiftYDTestLocation = frc::SmartDashboard::GetNumber("Set Position Y", 0);
  // VeLFT_Cnt_LiftXDTestLocation = frc::SmartDashboard::GetNumber("Set Position X", 0);

  // if((L_p != V_LiftPID_Gx[E_kP]))   { m_liftpidYD.SetP(L_p); m_liftpidXD.SetP(L_p); V_LiftPID_Gx[E_kP] = L_p; }
  // if((L_i != V_LiftPID_Gx[E_kI]))   { m_liftpidYD.SetI(L_i); m_liftpidXD.SetI(L_i); V_LiftPID_Gx[E_kI] = L_i; }
  // if((L_d != V_LiftPID_Gx[E_kD]))   { m_liftpidYD.SetD(L_d); m_liftpidXD.SetD(L_d); V_LiftPID_Gx[E_kD] = L_d; }
  // if((L_iz != V_LiftPID_Gx[E_kIz])) { m_liftpidYD.SetIZone(L_iz); m_liftpidXD.SetIZone(L_iz); V_LiftPID_Gx[E_kIz] = L_iz; }
  // if((L_ff != V_LiftPID_Gx[E_kFF])) { m_liftpidYD.SetFF(L_ff); m_liftpidXD.SetFF(L_ff); V_LiftPID_Gx[E_kFF] = L_ff; }
  // if((L_max != V_LiftPID_Gx[E_kMaxOutput]) || (L_min != V_LiftPID_Gx[E_kMinOutput])) { m_liftpidYD.SetOutputRange(L_min, L_max); m_liftpidXD.SetOutputRange(L_min, L_max); V_LiftPID_Gx[E_kMinOutput] = L_min; V_LiftPID_Gx[E_kMaxOutput] = L_max; }
  
  VaMAN_InS_RampRateMoterB[E_S0_Rest][VeMAN_Cnt_ManIterationNew]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_Rest][VeMAN_Cnt_ManIterationNew]",            VaMAN_InS_RampRateMoterB[E_S0_Rest][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S1_Intake][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S1_Intake][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterB[E_S1_Intake][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterB[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]",     VaMAN_InS_RampRateMoterB[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterB[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterB[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterB[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         VaMAN_InS_RampRateMoterB[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        VaMAN_InS_RampRateMoterB[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S0_Rest][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_Rest][E_LiftIteration2]",            VaMAN_InS_RampRateMoterB[E_S0_Rest][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S1_Intake][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S1_Intake][E_LiftIteration2]",      VaMAN_InS_RampRateMoterB[E_S1_Intake][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S2_TradeOff][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_TradeOff][E_LiftIteration2]",   VaMAN_InS_RampRateMoterB[E_S2_TradeOff][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S3_Swiper][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S3_Swiper][E_LiftIteration2]",     VaMAN_InS_RampRateMoterB[E_S3_Swiper][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S4_DrivingState][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S4_DrivingState][E_LiftIteration2]",   VaMAN_InS_RampRateMoterB[E_S4_DrivingState][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S5_Positioning][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S5_Positioning][E_LiftIteration2]",   VaMAN_InS_RampRateMoterB[E_S5_Positioning][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S6_DroppingTheLoot][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S6_DroppingTheLoot][E_LiftIteration2]",      VaMAN_InS_RampRateMoterB[E_S6_DroppingTheLoot][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         VaMAN_InS_RampRateMoterB[E_S10_final_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        VaMAN_InS_RampRateMoterB[E_S11_final_OWO][E_LiftIteration2]);

  VaMAN_InS_RampRateMoterA[E_S0_Rest][VeMAN_Cnt_ManIterationNew]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S0_Rest][VeMAN_Cnt_ManIterationNew]",            VaMAN_InS_RampRateMoterA[E_S0_Rest][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S1_Intake][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S1_Intake][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterA[E_S1_Intake][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterA[E_S2_TradeOff][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]",     VaMAN_InS_RampRateMoterA[E_S3_Swiper][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterA[E_S4_DrivingState][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterA[E_S5_Positioning][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterA[E_S6_DroppingTheLoot][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         VaMAN_InS_RampRateMoterA[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        VaMAN_InS_RampRateMoterA[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S0_Rest][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S0_Rest][E_LiftIteration2]",            VaMAN_InS_RampRateMoterA[E_S0_Rest][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S1_Intake][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S1_Intake][E_LiftIteration2]",      VaMAN_InS_RampRateMoterA[E_S1_Intake][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S2_TradeOff][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S2_TradeOff][E_LiftIteration2]",   VaMAN_InS_RampRateMoterA[E_S2_TradeOff][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S3_Swiper][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S3_Swiper][E_LiftIteration2]",     VaMAN_InS_RampRateMoterA[E_S3_Swiper][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S4_DrivingState][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S4_DrivingState][E_LiftIteration2]",   VaMAN_InS_RampRateMoterA[E_S4_DrivingState][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S5_Positioning][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S5_Positioning][E_LiftIteration2]",   VaMAN_InS_RampRateMoterA[E_S5_Positioning][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S6_DroppingTheLoot][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S6_DroppingTheLoot][E_LiftIteration2]",      VaMAN_InS_RampRateMoterA[E_S6_DroppingTheLoot][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]", VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]",         VaMAN_InS_RampRateMoterA[E_S10_final_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]",        VaMAN_InS_RampRateMoterA[E_S11_final_OWO][E_LiftIteration2]);
  #endif
  }

/******************************************************************************
 * Function:     LiftControlInit
 *
 * Description:  Initialization function for the lift control.
 ******************************************************************************/
void LiftControlInit()
  {
  T_Man_DoesStuffMaybe LeMAN_e_Index;

  VeMAN_CnT_Man_DoesStuffMaybe = E_S0_Rest;
  VeMAN_Cnt_ManIteration = VeMAN_Cnt_ManIterationNew;
  VeMAN_Cnt_LayoverTimer = 0;
  VeMAN_b_CriteriaMet = false;

  VeLFT_Cnt_CommandYD = 0;
  VeLFT_Cnt_CommandXD = 0;

  VeLFT_Cnt_LiftYDTestLocation = 0;
  VeLFT_Cnt_LiftXDTestLocation = 0;

  VeLFT_Cnt_LiftYDTestPowerCmnd = 0;
  VeLFT_Cnt_LiftXDTestPowerCmnd = 0;

  VeLFT_b_Paused = false;
  VeLFT_b_PausedXDPosition = 0;
  VeLFT_b_PausedYDPosition = 0;

  VeLFT_b_WaitingForDriverINS = false;

  VeLFT_b_LiftInitialized = false;

  for (LeMAN_e_Index = E_S0_Rest;
       LeMAN_e_Index < E_Man_State_Sz;
       LeMAN_e_Index = T_Man_DoesStuffMaybe(int(LeMAN_e_Index) + 1))
      {
      VaLFT_v_LiftMotorYDMaxCurrent[LeLFT_e_Index] = 0;
      VaLFT_v_LiftMotorXDMaxCurrent[LeLFT_e_Index] = 0;
      }
  }

/******************************************************************************
 * Function:     RecordLiftMotorMaxCurrent
 *
 * Description:  Record the max observed current.  
 *               This is for instrumentation only.
 ******************************************************************************/
void RecordManipulatorMotorMaxCurrent(T_Man_DoesStuffMaybe LeMAN_Cnt_CurrentState,                                
                               double       LeMAN_v_MotorCurrentOutA,
                               double       LeMAN_v_MotorCurrentOutB)
  {
  if (fabs(LeLFT_v_MotorYDCurrentOut) > fabs(VaLFT_v_LiftMotorYDMaxCurrent[LeLFT_Cnt_CurrentState]))
    {
    VaLFT_v_LiftMotorYDMaxCurrent[LeLFT_Cnt_CurrentState] = LeLFT_v_MotorYDCurrentOut;
    }
  
  if (fabs(LeLFT_v_MotorXDCurrentOut) > fabs(VaLFT_v_LiftMotorXDMaxCurrent[LeLFT_Cnt_CurrentState]))
    {
    VaLFT_v_LiftMotorXDMaxCurrent[LeLFT_Cnt_CurrentState] = LeLFT_v_MotorXDCurrentOut;
    }
  }

/******************************************************************************
 * Function:     Lift_Control_ManualOverride
 *
 * Description:  Manual override control used during the FRC test section.
 ******************************************************************************/
void Lift_Control_ManualOverride(double *LeLFT_Cmd_CommandYD,
                                 double *LeLFT_Cmd_CommandXD,
                                 double  LeLFT_v_MotorYDCurrentOut,
                                 double  LeLFT_v_MotorXDCurrentOut,
                                 TeLFT_e_LiftCmndDirection LeLFT_Cmd_DriverLiftDirection,
                                 bool    LeLFT_b_LimitDetectedYD,
                                 bool    LeLFT_b_LimitDetectedXD)
  {
  double LeMAN_v_MoterPowerA= 0;
  double LeMAN_v_MoterPowerB= 0;
  T_Man_DoesStuffMaybe LeMAN_Cnt_CurrentState = E_S0_Rest; // Not really the lift state, but allows us record the max currents

    if (LeLFT_Cmd_DriverLiftDirection == E_LiftCmndUp)
      {
      LeMAN_v_MoterPowerA= K_Manipulator_Driver_manual_MoterA;
      LeMAN_Cnt_CurrentState = E_S0_Rest;
      }
    else if ((LeLFT_Cmd_DriverLiftDirection == E_LiftCmndDown) &&
             (LeLFT_b_LimitDetectedYD == false))
      {
      LeMAN_v_MoterPowerA= K_Manipulator_Driver_manual_MoterB;
      LeMAN_Cnt_CurrentState = E_S1_Intake;
      }
    else if ((LeLFT_Cmd_DriverLiftDirection == E_LiftCmndBack) &&
             (LeLFT_b_LimitDetectedXD == false))
      {
      LeMAN_v_MoterPowerB= K_lift_driver_manual_back_XD;
      LeMAN_Cnt_CurrentState = E_S6_DroppingTheLoot;
      }
    else if (LeLFT_Cmd_DriverLiftDirection == E_LiftCmndForward)
      {
      LeMAN_v_MoterPowerB= K_lift_driver_manual_forward_XD;
      LeMAN_Cnt_CurrentState = E_S2_TradeOff;
      }

  RecordLiftMotorMaxCurrent(LeLFT_Cnt_CurrentState,                                
                            LeLFT_v_MotorYDCurrentOut,
                            LeLFT_v_MotorXDCurrentOut);

  *LeLFT_Cmd_CommandYD = LeLFT_v_LiftPowerYD;
  *LeLFT_Cmd_CommandXD = LeLFT_v_LiftPowerXD;
  }


/******************************************************************************
 * Function:     S2_lift_down_YD
 *
 * Description:  State 2: moving robert up by moving y-lift down
 ******************************************************************************/
 bool S2_lift_down_YD(double         LeLFT_b_AutoClimbButton,
                      double         LeLFT_In_MeasuredPositionYD,
                      double         LeLFT_In_MeasuredPositionXD,
                      double        *LeLFT_Cmd_CommandYD,
                      double        *LeLFT_Cmd_CommandXD,
                      double        *LeLFT_InS_CommandRateYD,
                      double        *LeLFT_InS_CommandRateXD,
                      T_Lift_Iteration LeLFT_Cmd_LiftIteration)
{
  bool LeLFT_b_CriteriaMet = false;

  *LeLFT_Cmd_CommandYD = K_lift_S2_YD;

  *LeLFT_Cmd_CommandXD = K_lift_min_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S1_Intake][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S1_Intake][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S2_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S2_YD - K_lift_deadband_YD)) {
    LeLFT_b_CriteriaMet = true;
  }

  return(LeLFT_b_CriteriaMet);
}

/******************************************************************************
 * Function:       S3_move_forward_XD,
 *
 * Description:  State 3: moving x lift haha it has to do its job
 ******************************************************************************/
 bool S3_move_forward_XD(double         LeLFT_b_AutoClimbButton,
                         double         LeLFT_In_MeasuredPositionYD,
                         double         LeLFT_In_MeasuredPositionXD,
                         double        *LeLFT_Cmd_CommandYD,
                         double        *LeLFT_Cmd_CommandXD,
                         double        *LeLFT_InS_CommandRateYD,
                         double        *LeLFT_InS_CommandRateXD,
                         T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeLFT_Cmd_CommandXD = K_lift_S3_XD;

  *LeLFT_Cmd_CommandYD = K_lift_S3_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S2_TradeOff][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S2_TradeOff][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S3_XD + K_lift_deadband_XD) && LeLFT_In_MeasuredPositionXD >= (K_lift_S3_XD - K_lift_deadband_XD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
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
 * Function:       S4_stretch_up_YD,
 *
 * Description:  State 4: x lift no move, y lift go
 ******************************************************************************/
 bool S4_stretch_up_YD(double         LeLFT_b_AutoClimbButton,
                       double         LeLFT_In_MeasuredPositionYD,
                       double         LeLFT_In_MeasuredPositionXD,
                       double        *LeLFT_Cmd_CommandYD,
                       double        *LeLFT_Cmd_CommandXD,
                       double        *LeLFT_InS_CommandRateYD,
                       double        *LeLFT_InS_CommandRateXD,
                       T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
   bool LeLFT_b_CriteriaMet = false;
  
  *LeLFT_Cmd_CommandYD = K_lift_S4_YD;

  *LeLFT_Cmd_CommandXD = K_lift_S4_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S3_Swiper][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S3_Swiper][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S4_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S4_YD - K_lift_deadband_YD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
      VeLFT_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
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
 * Function:       S5_more_forward_XD,
 *
 * Description:  State 5: y lift no move, x lift go
 ******************************************************************************/
 bool S5_more_forward_XD(double         LeLFT_b_AutoClimbButton,
                         double         LeLFT_In_MeasuredPositionYD,
                         double         LeLFT_In_MeasuredPositionXD,
                         double        *LeLFT_Cmd_CommandYD,
                         double        *LeLFT_Cmd_CommandXD,
                         double        *LeLFT_InS_CommandRateYD,
                         double        *LeLFT_InS_CommandRateXD,
                         T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeLFT_Cmd_CommandXD = K_lift_S5_XD;

  *LeLFT_Cmd_CommandYD = K_lift_S5_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S4_DrivingState][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S4_DrivingState][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S5_XD + K_lift_deadband_XD) && LeLFT_In_MeasuredPositionXD >= (K_lift_S5_XD - K_lift_deadband_XD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
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
 * Function:       S6_lift_up_more_YD,
 *
 * Description:  State 6: y lift go down, x lift bad stop what's in your mouth no get back here doN'T EAT IT
 ******************************************************************************/
 bool S6_lift_up_more_YD(double         LeLFT_b_AutoClimbButton,
                         double         LeLFT_In_MeasuredPositionYD,
                         double         LeLFT_In_MeasuredPositionXD,
                         double        *LeLFT_Cmd_CommandYD,
                         double        *LeLFT_Cmd_CommandXD,
                         double        *LeLFT_InS_CommandRateYD,
                         double        *LeLFT_InS_CommandRateXD,
                         T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeLFT_Cmd_CommandYD = K_lift_S6_YD;

  *LeLFT_Cmd_CommandXD = K_lift_S6_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S5_Positioning][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S5_Positioning][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S6_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S6_YD - K_lift_deadband_YD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
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
 bool S7_move_back_XD(double         LeLFT_b_AutoClimbButton,
                      double         LeLFT_In_MeasuredPositionYD,
                      double         LeLFT_In_MeasuredPositionXD,
                      double         LeLEFT_Deg_GyroAngleYaws,
                      double        *LeLFT_Cmd_CommandYD,
                      double        *LeLFT_Cmd_CommandXD,
                      double        *LeLFT_InS_CommandRateYD,
                      double        *LeLFT_InS_CommandRateXD,
                      T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeLFT_Cmd_CommandXD = K_lift_S7_XD;

  *LeLFT_Cmd_CommandYD = K_lift_S7_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S6_DroppingTheLoot][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S6_DroppingTheLoot][LeLFT_Cmd_LiftIteration]; // Don't go too fast, going slower will help to reduce rocking

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S7_XD + K_lift_deadband_XD)  && LeLFT_In_MeasuredPositionXD >= (K_lift_S7_XD - K_lift_deadband_XD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
      VeLFT_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
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
 bool S8_more_down_some_YD(double         LeLFT_b_AutoClimbButton,
                           double         LeLFT_In_MeasuredPositionYD,
                           double         LeLFT_In_MeasuredPositionXD,
                           double        *LeLFT_Cmd_CommandYD,
                           double        *LeLFT_Cmd_CommandXD,
                           double        *LeLFT_InS_CommandRateYD,
                           double        *LeLFT_InS_CommandRateXD,
                           T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeLFT_Cmd_CommandYD = K_lift_S8_YD;

  *LeLFT_Cmd_CommandXD = K_lift_S8_XD;

  *LeLFT_InS_CommandRateYD = VaLFT_InS_LiftRampRateYD[E_S8_more_down_some_YD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaLFT_InS_LiftRampRateXD[E_S8_more_down_some_YD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S8_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S8_YD - K_lift_deadband_YD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
      VeLFT_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
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
 * Function:       S9_back_rest_XD
 *
 * Description:  State 9: reset it to initial x position (we aren't fixing my back  :(  )
 ******************************************************************************/
 bool S9_back_rest_XD(double         LeLFT_b_AutoClimbButton,
                      double         LeLFT_In_MeasuredPositionYD,
                      double         LeLFT_In_MeasuredPositionXD,
                      double        *LeLFT_Cmd_CommandYD,
                      double        *LeLFT_Cmd_CommandXD,
                      double        *LeLFT_InS_CommandRateYD,
                      double        *LeLFT_InS_CommandRateXD,
                      T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeLFT_Cmd_CommandXD = K_lift_S9_XD;

  *LeLFT_Cmd_CommandYD = K_lift_S9_YD;

  *LeLFT_InS_CommandRateYD = VaLFT_InS_LiftRampRateYD[E_S9_back_rest_XD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaLFT_InS_LiftRampRateXD[E_S9_back_rest_XD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S9_XD + K_lift_deadband_XD) && LeLFT_In_MeasuredPositionXD >= (K_lift_S9_XD - K_lift_deadband_XD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
      VeLFT_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
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
 * Function:       S10_final_YD
 *
 * Description:  State 10: y move down, robert move up (what a chad)
 ******************************************************************************/
 bool S10_final_YD(double         LeLFT_b_AutoClimbButton,
                   double         LeLFT_In_MeasuredPositionYD,
                   double         LeLFT_In_MeasuredPositionXD,
                   double        *LeLFT_Cmd_CommandYD,
                   double        *LeLFT_Cmd_CommandXD,
                   double        *LeLFT_InS_CommandRateYD,
                   double        *LeLFT_InS_CommandRateXD,
                   T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeLFT_Cmd_CommandYD = K_lift_S10_YD;

  *LeLFT_Cmd_CommandXD = K_lift_S10_XD;

  *LeLFT_InS_CommandRateYD = VaLFT_InS_LiftRampRateYD[E_S10_final_YD][LeLFT_Cmd_LiftIteration]; // Slow down, don't yank too hard

  *LeLFT_InS_CommandRateXD = VaLFT_InS_LiftRampRateXD[E_S10_final_YD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S10_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S10_YD - K_lift_deadband_YD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
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
 * Function:       S11_final_OWO
 *
 * Description:  State 11: uwu
 ******************************************************************************/
 bool S11_final_OWO(double         LeLFT_b_AutoClimbButton,
                    double         LeLFT_In_MeasuredPositionYD,
                    double         LeLFT_In_MeasuredPositionXD,
                    double        *LeLFT_Cmd_CommandYD,
                    double        *LeLFT_Cmd_CommandXD,
                    double        *LeLFT_InS_CommandRateYD,
                    double        *LeLFT_InS_CommandRateXD,
                    T_Lift_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeLFT_Cmd_CommandYD = K_lift_S11_YD;

  *LeLFT_Cmd_CommandXD = K_lift_S11_XD;

  *LeLFT_InS_CommandRateYD = VaLFT_InS_LiftRampRateYD[E_S11_final_OWO][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaLFT_InS_LiftRampRateXD[E_S11_final_OWO][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S11_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S11_YD - K_lift_deadband_YD)) {
    VeLFT_Cnt_LiftDebounceTimer += C_ExeTime;
    if (VeLFT_Cnt_LiftDebounceTimer >= K_Lift_deadband_timer){
      VeLFT_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
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
 * Function:     ManipulatorControlDictator
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/
T_Man_DoesStuffMaybe ManipulatorControlDictator(bool                LeLFT_b_AutoClimbButton,
                                   bool                LeLFT_b_DriverAutoClimbPause,
                                   TeLFT_e_LiftCmndDirection LeLFT_Cmd_DriverLiftDirection,
                                   double              LeLFT_SEC_GameTime,
                                   T_Man_DoesStuffMaybe        LeMAN_Cnt_CurrentState,                                
                                   double              LeLFT_In_MeasuredPositionYD,
                                   double              LeLFT_In_MeasuredPositionXD,
                                   double             *LeLFT_Cmd_CommandYD,
                                   double             *LeLFT_Cmd_CommandXD,
                                   double             *LeLFT_Pct_CommandPwrYD,
                                   double             *LeLFT_Pct_CommandPwrXD,
                                   bool                LeLFT_b_LimitDetectedYD,
                                   bool                LeLFT_b_LimitDetectedXD,
                                   double              LeLEFT_Deg_GyroAngleYaws,
                                   double              LeLFT_v_MotorYDCurrentOut,
                                   double              LeLFT_v_MotorXDCurrentOut,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD)
  {
  T_Man_DoesStuffMaybe LeLFT_e_CommandedState = LeMAN_Cnt_CurrentState;
  double LeMAN_Cmd_CommandA_Temp = 0;
  double LeMAN_Cmd_CommandB_Temp = 0;
  double LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S0_Rest][VeMAN_Cnt_ManIterationNew];
  double LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S0_Rest][VeMAN_Cnt_ManIterationNew];
  double LeMAN_v_MoterPowerA= 0;
  double LeMAN_v_MoterPowerB= 0;

  if (VeMAN_b_MoterTestA == true)
    {
    /* Only used for testing. */
    LeLFT_Cmd_CommandYD_Temp = VeLFT_Cnt_LiftYDTestLocation;
    LeLFT_Cmd_CommandXD_Temp = VeLFT_Cnt_LiftXDTestLocation;
    }
  else if (VeLFT_b_LiftInitialized == false)
    {
    if (LeLFT_b_LimitDetectedYD == false)
      {
      LeLFT_v_LiftPowerYD = K_lift_autoResetDown_YD;
      }

    if (LeLFT_b_LimitDetectedXD == false)
      {
      LeLFT_v_LiftPowerXD = K_lift_driver_manual_back_XD;
      }
    
    if (LeLFT_b_LimitDetectedYD == true && 
        LeLFT_b_LimitDetectedXD == true)
      {
      LeLFT_v_LiftPowerYD = 0;
      LeLFT_v_LiftPowerXD = 0;
      VeLFT_b_LiftInitialized = true;

      // EncodersLiftInit(m_encoderLiftYD,
      //                  m_encoderLiftXD);
      }
    }
  else if ((LeLFT_b_DriverAutoClimbPause == true) && (VeLFT_b_Paused == false))
    {
    /* The driver pressed a button to puase the climb process.  Let's save the current locations and hold. */
    VeLFT_b_Paused = true;
    VeLFT_b_PausedXDPosition = LeLFT_In_MeasuredPositionXD;
    VeLFT_b_PausedYDPosition = LeLFT_In_MeasuredPositionYD;
    /* Set commanded location to current measured location for this loop. */
    LeLFT_Cmd_CommandXD_Temp = LeLFT_In_MeasuredPositionXD;
    LeLFT_Cmd_CommandYD_Temp = LeLFT_In_MeasuredPositionYD;
    }
  else if (((LeLFT_b_AutoClimbButton == true) && (VeLFT_b_Paused == true)) || 
            (VeLFT_b_Paused == false))
    {
    VeLFT_b_Paused = false;
    VeLFT_b_PausedXDPosition = LeLFT_In_MeasuredPositionXD;
    VeLFT_b_PausedYDPosition = LeLFT_In_MeasuredPositionYD;

    switch (LeLFT_Cnt_CurrentState)
      {
        case E_S0_Rest:
            if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndUp)
              {
              LeLFT_Cmd_CommandYD_Temp = *LeLFT_Cmd_CommandYD + K_lift_driver_up_rate_YD;
              }
            else if (LeLFT_Cmd_DriverLiftDirection == E_LiftCmndDown)
              {
              LeLFT_Cmd_CommandYD_Temp = *LeLFT_Cmd_CommandYD - K_lift_driver_down_rate_YD;
              }
              else 
              {
                LeLFT_Cmd_CommandYD_Temp = *LeLFT_Cmd_CommandYD;
              }
            /* The driver should only initiate the state machine once the robot has become suspended. */
            if (LeLFT_b_AutoClimbButton == true && LeLFT_In_MeasuredPositionYD >= K_lift_enable_auto_YD) {
                LeLFT_e_CommandedState = E_S1_Intake;
            }
        break;

        case E_S1_Intake:
            VeMAN_b_CriteriaMet = S2_lift_down_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S2_TradeOff;
            }
        break;

        case E_S2_TradeOff:
            VeMAN_b_CriteriaMet = S3_move_forward_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S3_Swiper;
            }
        break;

        case E_S3_Swiper:
            VeMAN_b_CriteriaMet = S4_stretch_up_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S4_DrivingState;
            }
        break;

        case E_S4_DrivingState:
            VeMAN_b_CriteriaMet = S5_more_forward_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S5_Positioning;
            }
        break;

        case E_S5_Positioning:
            VeMAN_b_CriteriaMet = S6_lift_up_more_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S6_DroppingTheLoot;
            }
        break;

        case E_S6_DroppingTheLoot:
            VeMAN_b_CriteriaMet = S7_move_back_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, LeLEFT_Deg_GyroAngleYaws, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S8_more_down_some_YD;
            }
        break;

        case E_S8_more_down_some_YD:
            VeLFT_b_CriteriaMet = S8_more_down_some_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeLFT_Cmd_CommandYD_Temp, &LeLFT_Cmd_CommandXD_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeLFT_Cnt_LiftIteration);
            if(VeLFT_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S9_back_rest_XD;
            }
        break;

        case E_S9_back_rest_XD:
            VeLFT_b_CriteriaMet = S9_back_rest_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeLFT_Cmd_CommandYD_Temp, &LeLFT_Cmd_CommandXD_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeLFT_Cnt_LiftIteration);
            if(VeLFT_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S10_final_YD;
            }
        break;

        case E_S10_final_YD:
            VeLFT_b_CriteriaMet = S10_final_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeLFT_Cmd_CommandYD_Temp, &LeLFT_Cmd_CommandXD_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeLFT_Cnt_LiftIteration);
            if(VeLFT_b_CriteriaMet == true){
              LeLFT_e_CommandedState = E_S11_final_OWO;
            }
        break;

        case E_S11_final_OWO:
            VeMAN_b_CriteriaMet = S11_final_OWO(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true &&VeMAN_Cnt_ManIteration < E_LiftIteration2){
              LeLFT_e_CommandedState = E_S1_Intake;
             VeMAN_Cnt_ManIteration = E_LiftIteration2;
            }
            else if(VeLFT_b_CriteriaMet == true &&VeLFT_Cnt_LiftIteration >= E_LiftIteration2){
              LeLFT_e_CommandedState = E_S11_final_OWO;
            }
        break;
      }
    }
  else
    {
    /* Lift is currently paused: */
    LeLFT_Cmd_CommandXD_Temp = VeLFT_b_PausedXDPosition;
    LeLFT_Cmd_CommandYD_Temp = VeLFT_b_PausedYDPosition;
    }

  /* Place limits on the travel of XD and YD to prevent damage: */
  if (LeLFT_Cmd_CommandYD_Temp > K_lift_max_YD)
    {
    LeLFT_Cmd_CommandYD_Temp = K_lift_max_YD;
    }
  else if (LeLFT_Cmd_CommandYD_Temp < K_lift_min_YD)
    {
    LeLFT_Cmd_CommandYD_Temp = K_lift_max_YD;
    }

  if (LeLFT_Cmd_CommandXD_Temp > K_lift_max_XD)
    {
    LeLFT_Cmd_CommandXD_Temp = K_lift_max_XD;
    }
  else if (LeLFT_Cmd_CommandXD_Temp < K_lift_min_XD)
    {
    LeLFT_Cmd_CommandXD_Temp = K_lift_max_XD;
    }

  *LeLFT_Cmd_CommandYD= RampTo(LeLFT_Cmd_CommandYD_Temp, *LeLFT_Cmd_CommandYD, LeLFT_InS_CommandRateYD);

  *LeLFT_Cmd_CommandXD= RampTo(LeLFT_Cmd_CommandXD_Temp, *LeLFT_Cmd_CommandXD, LeLFT_InS_CommandRateXD);

  *LeLFT_Pct_CommandPwrYD = LeLFT_v_LiftPowerYD;
  
  *LeLFT_Pct_CommandPwrXD = LeLFT_v_LiftPowerXD;

  RecordLiftMotorMaxCurrent(LeLFT_Cnt_CurrentState,
                            LeLFT_v_MotorYDCurrentOut,
                            LeLFT_v_MotorXDCurrentOut);

  return(LeLFT_e_CommandedState);
}

#endif