/*
  Lift.cpp

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

T_Lift_State V_Lift_state = E_S0_BEGONE;
T_Lift_Iteration V_LiftIteration = E_LiftIteration1;

double V_LiftDebounceTimer = 0; // owo, because Chloe
bool   V_criteria_met = false;
bool   V_LiftInitialized = false;

double V_lift_command_YD = 0;
double V_lift_command_XD = 0;

double V_LiftYD_TestLocation = 0;
double V_LiftXD_TestLocation = 0;

double V_LiftYD_TestPowerCmnd = 0;
double V_LiftXD_TestPowerCmnd = 0;

double V_LiftMotorYD_MaxCurrent[E_Lift_State_Sz];
double V_LiftMotorXD_MaxCurrent[E_Lift_State_Sz];

bool   V_Lift_WaitingForDriverINS = false;  // Instrumentation only, but indication that we are waiting for the driver to press button for next step.
bool   V_Lift_Paused = false;
double V_Lift_PausedXD_Position = 0;
double V_Lift_PausedYD_Position = 0;

double KV_LiftRampRateYD[E_Lift_State_Sz][E_LiftIterationSz];
double KV_LiftRampRateXD[E_Lift_State_Sz][E_LiftIterationSz];

#ifdef LiftXY_Test
bool   V_LiftXY_Test = false; // temporary, we don't want to use the manual overrides
double V_LiftPID_Gx[E_PID_SparkMaxCalSz];
#else
bool V_LiftXY_Test = false;
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
  T_Lift_State     L_Index1;
  T_Lift_Iteration L_Index2;

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

  for (L_Index2 = E_LiftIteration1;
       L_Index2 < E_LiftIterationSz;
       L_Index2 = T_Lift_Iteration(int(L_Index2) + 1))
      {
      for (L_Index1 = E_S0_BEGONE;
           L_Index1 < E_Lift_State_Sz;
           L_Index1 = T_Lift_State(int(L_Index1) + 1))
          {
          KV_LiftRampRateYD[L_Index1][L_Index2] = K_LiftRampRateYD[L_Index1][L_Index2];
          KV_LiftRampRateXD[L_Index1][L_Index2] = K_LiftRampRateXD[L_Index1][L_Index2];
          }
      }
  
  #ifdef LiftXY_Test
  T_PID_SparkMaxCal L_Index = E_kP;

  for (L_Index = E_kP;
       L_Index < E_PID_SparkMaxCalSz;
       L_Index = T_PID_SparkMaxCal(int(L_Index) + 1))
      {
      V_LiftPID_Gx[L_Index] = K_LiftPID_Gx[L_Index];
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

  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]",            K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]",      K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration1]",   K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration1]",     K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration1]",   K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration1]",   K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration1]",      K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration1]", K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration1]",      K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration1]",         K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration1]",        K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration2]",            K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration2]",      K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration2]",   K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration2]",     K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration2]",   K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration2]",   K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration2]",      K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]);

  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1]",            K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration1]",      K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration1]",   K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration1]",     K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration1]",   K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration1]",   K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration1]",      K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration1]", K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration1]",      K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration1]",         K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration1]",        K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration2]",            K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration2]",      K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration2]",   K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration2]",     K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration2]",   K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration2]",   K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration2]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration2]",      K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration2]);
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

  // V_LiftYD_TestLocation = frc::SmartDashboard::GetNumber("Set Position Y", 0);
  // V_LiftXD_TestLocation = frc::SmartDashboard::GetNumber("Set Position X", 0);

  // if((L_p != V_LiftPID_Gx[E_kP]))   { m_liftpidYD.SetP(L_p); m_liftpidXD.SetP(L_p); V_LiftPID_Gx[E_kP] = L_p; }
  // if((L_i != V_LiftPID_Gx[E_kI]))   { m_liftpidYD.SetI(L_i); m_liftpidXD.SetI(L_i); V_LiftPID_Gx[E_kI] = L_i; }
  // if((L_d != V_LiftPID_Gx[E_kD]))   { m_liftpidYD.SetD(L_d); m_liftpidXD.SetD(L_d); V_LiftPID_Gx[E_kD] = L_d; }
  // if((L_iz != V_LiftPID_Gx[E_kIz])) { m_liftpidYD.SetIZone(L_iz); m_liftpidXD.SetIZone(L_iz); V_LiftPID_Gx[E_kIz] = L_iz; }
  // if((L_ff != V_LiftPID_Gx[E_kFF])) { m_liftpidYD.SetFF(L_ff); m_liftpidXD.SetFF(L_ff); V_LiftPID_Gx[E_kFF] = L_ff; }
  // if((L_max != V_LiftPID_Gx[E_kMaxOutput]) || (L_min != V_LiftPID_Gx[E_kMinOutput])) { m_liftpidYD.SetOutputRange(L_min, L_max); m_liftpidXD.SetOutputRange(L_min, L_max); V_LiftPID_Gx[E_kMinOutput] = L_min; V_LiftPID_Gx[E_kMaxOutput] = L_max; }
  
  KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]",            KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]",      KV_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration1]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration1]",   KV_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration1]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration1]",     KV_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration1]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration1]",   KV_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration1]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration1]",   KV_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration1]",      KV_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration1] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration1]", KV_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration1]",      KV_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S10_final_YD][E_LiftIteration1]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration1]",         KV_LiftRampRateXD[E_S10_final_YD][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration1]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration1]",        KV_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration1]);
  KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration2]",            KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration2]",      KV_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration2]",   KV_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration2]",     KV_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration2]",   KV_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration2]",   KV_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration2]",      KV_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", KV_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      KV_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         KV_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]);
  KV_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        KV_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]);

  KV_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1]",            KV_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration1]",      KV_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration1]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration1]",   KV_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration1]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration1]",     KV_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration1]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration1]",   KV_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration1]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration1]",   KV_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration1]",      KV_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration1] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration1]", KV_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration1]",      KV_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S10_final_YD][E_LiftIteration1]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration1]",         KV_LiftRampRateYD[E_S10_final_YD][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration1]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration1]",        KV_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration1]);
  KV_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration2]",            KV_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration2]",      KV_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration2]",   KV_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration2]",     KV_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration2]",   KV_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration2]",   KV_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration2]",      KV_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]", KV_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]",      KV_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]",         KV_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]);
  KV_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]",        KV_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]);
  #endif
  }

/******************************************************************************
 * Function:     LiftControlInit
 *
 * Description:  Initialization function for the lift control.
 ******************************************************************************/
void LiftControlInit()
  {
  T_Lift_State L_Index;

  V_Lift_state = E_S0_BEGONE;
  V_LiftIteration = E_LiftIteration1;
  V_LiftDebounceTimer = 0;
  V_criteria_met = false;

  V_lift_command_YD = 0;
  V_lift_command_XD = 0;

  V_LiftYD_TestLocation = 0;
  V_LiftXD_TestLocation = 0;

  V_LiftYD_TestPowerCmnd = 0;
  V_LiftXD_TestPowerCmnd = 0;

  V_Lift_Paused = false;
  V_Lift_PausedXD_Position = 0;
  V_Lift_PausedYD_Position = 0;

  V_Lift_WaitingForDriverINS = false;

  V_LiftInitialized = false;

  for (L_Index = E_S0_BEGONE;
       L_Index < E_Lift_State_Sz;
       L_Index = T_Lift_State(int(L_Index) + 1))
      {
      V_LiftMotorYD_MaxCurrent[L_Index] = 0;
      V_LiftMotorXD_MaxCurrent[L_Index] = 0;
      }
  }

/******************************************************************************
 * Function:     RecordLiftMotorMaxCurrent
 *
 * Description:  Record the max observed current.  
 *               This is for instrumentation only.
 ******************************************************************************/
void RecordLiftMotorMaxCurrent(T_Lift_State L_current_state,                                
                               double       L_liftMotorYD_CurrentOut,
                               double       L_liftMotorXD_CurrentOut)
  {
  if (fabs(L_liftMotorYD_CurrentOut) > fabs(V_LiftMotorYD_MaxCurrent[L_current_state]))
    {
    V_LiftMotorYD_MaxCurrent[L_current_state] = L_liftMotorYD_CurrentOut;
    }
  
  if (fabs(L_liftMotorXD_CurrentOut) > fabs(V_LiftMotorXD_MaxCurrent[L_current_state]))
    {
    V_LiftMotorXD_MaxCurrent[L_current_state] = L_liftMotorXD_CurrentOut;
    }
  }

/******************************************************************************
 * Function:     Lift_Control_ManualOverride
 *
 * Description:  Manual override control used during the FRC test section.
 ******************************************************************************/
void Lift_Control_ManualOverride(double *L_lift_command_YD,
                                 double *L_lift_command_XD,
                                 double  L_liftMotorYD_CurrentOut,
                                 double  L_liftMotorXD_CurrentOut,
                                 T_LiftCmndDirection L_DriverLiftCmndDirection,
                                 bool    L_YD_LimitDetected,
                                 bool    L_XD_LimitDetected)
  {
  double L_LiftYD_Power = 0;
  double L_LiftXD_Power = 0;
  T_Lift_State L_current_state = E_S0_BEGONE; // Not really the lift state, but allows us record the max currents

    if (L_DriverLiftCmndDirection == E_LiftCmndUp)
      {
      L_LiftYD_Power = K_lift_driver_manual_up_YD;
      L_current_state = E_S0_BEGONE;
      }
    else if ((L_DriverLiftCmndDirection == E_LiftCmndDown) &&
             (L_YD_LimitDetected == false))
      {
      L_LiftYD_Power = K_lift_driver_manual_down_YD;
      L_current_state = E_S2_lift_down_YD;
      }
    else if ((L_DriverLiftCmndDirection == E_LiftCmndBack) &&
             (L_XD_LimitDetected == false))
      {
      L_LiftXD_Power = K_lift_driver_manual_back_XD;
      L_current_state = E_S7_move_back_XD;
      }
    else if (L_DriverLiftCmndDirection == E_LiftCmndForward)
      {
      L_LiftXD_Power = K_lift_driver_manual_forward_XD;
      L_current_state = E_S3_move_forward_XD;
      }

  RecordLiftMotorMaxCurrent(L_current_state,                                
                            L_liftMotorYD_CurrentOut,
                            L_liftMotorXD_CurrentOut);

  *L_lift_command_YD = L_LiftYD_Power;
  *L_lift_command_XD = L_LiftXD_Power;
  }


/******************************************************************************
 * Function:     S2_lift_down_YD
 *
 * Description:  State 2: moving robert up by moving y-lift down
 ******************************************************************************/
 bool S2_lift_down_YD(double         L_driver_auto_climb_button,
                      double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD,
                      double        *L_lift_command_rate_YD,
                      double        *L_lift_command_rate_XD,
                      T_Lift_Iteration L_LiftIteration)
{
  bool L_criteria_met = false;

  *L_lift_command_YD = K_lift_S2_YD;

  *L_lift_command_XD = K_lift_min_XD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S2_lift_down_YD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S2_lift_down_YD][L_LiftIteration];

  if (L_lift_measured_position_YD <= (K_lift_S2_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S2_YD - K_lift_deadband_YD)) {
    L_criteria_met = true;
  }

  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S3_move_forward_XD,
 *
 * Description:  State 3: moving x lift haha it has to do its job
 ******************************************************************************/
 bool S3_move_forward_XD(double         L_driver_auto_climb_button,
                         double         L_lift_measured_position_YD,
                         double         L_lift_measured_position_XD,
                         double        *L_lift_command_YD,
                         double        *L_lift_command_XD,
                         double        *L_lift_command_rate_YD,
                         double        *L_lift_command_rate_XD,
                         T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;
  
  *L_lift_command_XD = K_lift_S3_XD;

  *L_lift_command_YD = K_lift_S3_YD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S3_move_forward_XD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S3_move_forward_XD][L_LiftIteration];

  if (L_lift_measured_position_XD <= (K_lift_S3_XD + K_lift_deadband_XD) && L_lift_measured_position_XD >= (K_lift_S3_XD - K_lift_deadband_XD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }

  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S4_stretch_up_YD,
 *
 * Description:  State 4: x lift no move, y lift go
 ******************************************************************************/
 bool S4_stretch_up_YD(double         L_driver_auto_climb_button,
                       double         L_lift_measured_position_YD,
                       double         L_lift_measured_position_XD,
                       double        *L_lift_command_YD,
                       double        *L_lift_command_XD,
                       double        *L_lift_command_rate_YD,
                       double        *L_lift_command_rate_XD,
                       T_Lift_Iteration L_LiftIteration)  
{
   bool L_criteria_met = false;
  
  *L_lift_command_YD = K_lift_S4_YD;

  *L_lift_command_XD = K_lift_S4_XD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S4_stretch_up_YD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S4_stretch_up_YD][L_LiftIteration];

  if (L_lift_measured_position_YD <= (K_lift_S4_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S4_YD - K_lift_deadband_YD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
      V_Lift_WaitingForDriverINS = true;
      if (L_driver_auto_climb_button == true){
         /* Let the driver determine when we are not swinging and can proceed */
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
         V_Lift_WaitingForDriverINS = false;
      }
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S5_more_forward_XD,
 *
 * Description:  State 5: y lift no move, x lift go
 ******************************************************************************/
 bool S5_more_forward_XD(double         L_driver_auto_climb_button,
                         double         L_lift_measured_position_YD,
                         double         L_lift_measured_position_XD,
                         double        *L_lift_command_YD,
                         double        *L_lift_command_XD,
                         double        *L_lift_command_rate_YD,
                         double        *L_lift_command_rate_XD,
                         T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;

  *L_lift_command_XD = K_lift_S5_XD;

  *L_lift_command_YD = K_lift_S5_YD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S5_more_forward_XD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S5_more_forward_XD][L_LiftIteration];

  if (L_lift_measured_position_XD <= (K_lift_S5_XD + K_lift_deadband_XD) && L_lift_measured_position_XD >= (K_lift_S5_XD - K_lift_deadband_XD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S6_lift_up_more_YD,
 *
 * Description:  State 6: y lift go down, x lift bad stop what's in your mouth no get back here doN'T EAT IT
 ******************************************************************************/
 bool S6_lift_up_more_YD(double         L_driver_auto_climb_button,
                         double         L_lift_measured_position_YD,
                         double         L_lift_measured_position_XD,
                         double        *L_lift_command_YD,
                         double        *L_lift_command_XD,
                         double        *L_lift_command_rate_YD,
                         double        *L_lift_command_rate_XD,
                         T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;

  *L_lift_command_YD = K_lift_S6_YD;

  *L_lift_command_XD = K_lift_S6_XD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S6_lift_up_more_YD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S6_lift_up_more_YD][L_LiftIteration];

  if (L_lift_measured_position_YD <= (K_lift_S6_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S6_YD - K_lift_deadband_YD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S7_move_back_XD,
 *
 * Description:  State 7: X go back-aroni, we look at gyro to make sure we aren't tilted too much
 ******************************************************************************/
 bool S7_move_back_XD(double         L_driver_auto_climb_button,
                      double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double         L_gyro_yawangledegrees,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD,
                      double        *L_lift_command_rate_YD,
                      double        *L_lift_command_rate_XD,
                      T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;

  *L_lift_command_XD = K_lift_S7_XD;

  *L_lift_command_YD = K_lift_S7_YD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S7_move_back_XD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S7_move_back_XD][L_LiftIteration]; // Don't go too fast, going slower will help to reduce rocking

  if (L_lift_measured_position_XD <= (K_lift_S7_XD + K_lift_deadband_XD)  && L_lift_measured_position_XD >= (K_lift_S7_XD - K_lift_deadband_XD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
      V_Lift_WaitingForDriverINS = true;
      if (L_driver_auto_climb_button == true){
         /* Let the driver determine when we are not swinging and can proceed */
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
         V_Lift_WaitingForDriverINS = false;
      }
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S8_more_down_some_YD,
 *
 * Description:  State 8: me when the lift go down more
 ******************************************************************************/
 bool S8_more_down_some_YD(double         L_driver_auto_climb_button,
                           double         L_lift_measured_position_YD,
                           double         L_lift_measured_position_XD,
                           double        *L_lift_command_YD,
                           double        *L_lift_command_XD,
                           double        *L_lift_command_rate_YD,
                           double        *L_lift_command_rate_XD,
                           T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;
  
  *L_lift_command_YD = K_lift_S8_YD;

  *L_lift_command_XD = K_lift_S8_XD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S8_more_down_some_YD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S8_more_down_some_YD][L_LiftIteration];

  if (L_lift_measured_position_YD <= (K_lift_S8_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S8_YD - K_lift_deadband_YD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
      V_Lift_WaitingForDriverINS = true;
      if (L_driver_auto_climb_button == true){
         /* Let the driver determine when we are not swinging and can proceed */
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
         V_Lift_WaitingForDriverINS = false;
      }
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S9_back_rest_XD
 *
 * Description:  State 9: reset it to initial x position (we aren't fixing my back  :(  )
 ******************************************************************************/
 bool S9_back_rest_XD(double         L_driver_auto_climb_button,
                      double         L_lift_measured_position_YD,
                      double         L_lift_measured_position_XD,
                      double        *L_lift_command_YD,
                      double        *L_lift_command_XD,
                      double        *L_lift_command_rate_YD,
                      double        *L_lift_command_rate_XD,
                      T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;
  
  *L_lift_command_XD = K_lift_S9_XD;

  *L_lift_command_YD = K_lift_S9_YD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S9_back_rest_XD][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S9_back_rest_XD][L_LiftIteration];

  if (L_lift_measured_position_XD <= (K_lift_S9_XD + K_lift_deadband_XD) && L_lift_measured_position_XD >= (K_lift_S9_XD - K_lift_deadband_XD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
      V_Lift_WaitingForDriverINS = true;
      if (L_driver_auto_climb_button == true){
         /* Let the driver determine when we are not swinging and can proceed */
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
         V_Lift_WaitingForDriverINS = false;
      }
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S10_final_YD
 *
 * Description:  State 10: y move down, robert move up (what a chad)
 ******************************************************************************/
 bool S10_final_YD(double         L_driver_auto_climb_button,
                   double         L_lift_measured_position_YD,
                   double         L_lift_measured_position_XD,
                   double        *L_lift_command_YD,
                   double        *L_lift_command_XD,
                   double        *L_lift_command_rate_YD,
                   double        *L_lift_command_rate_XD,
                   T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;
  
  *L_lift_command_YD = K_lift_S10_YD;

  *L_lift_command_XD = K_lift_S10_XD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S10_final_YD][L_LiftIteration]; // Slow down, don't yank too hard

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S10_final_YD][L_LiftIteration];

  if (L_lift_measured_position_YD <= (K_lift_S10_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S10_YD - K_lift_deadband_YD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}

/******************************************************************************
 * Function:       S11_final_OWO
 *
 * Description:  State 11: uwu
 ******************************************************************************/
 bool S11_final_OWO(double         L_driver_auto_climb_button,
                    double         L_lift_measured_position_YD,
                    double         L_lift_measured_position_XD,
                    double        *L_lift_command_YD,
                    double        *L_lift_command_XD,
                    double        *L_lift_command_rate_YD,
                    double        *L_lift_command_rate_XD,
                    T_Lift_Iteration L_LiftIteration)  
{
  bool L_criteria_met = false;
  
  *L_lift_command_YD = K_lift_S11_YD;

  *L_lift_command_XD = K_lift_S11_XD;

  *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S11_final_OWO][L_LiftIteration];

  *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S11_final_OWO][L_LiftIteration];

  if (L_lift_measured_position_YD <= (K_lift_S11_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S11_YD - K_lift_deadband_YD)) {
    V_LiftDebounceTimer += C_ExeTime;
    if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
      V_Lift_WaitingForDriverINS = true;
      if (L_driver_auto_climb_button == true){
         /* Let the driver determine when we are not swinging and can proceed */
         L_criteria_met = true;
         V_LiftDebounceTimer = 0;
         V_Lift_WaitingForDriverINS = false;
      }
    }
  }
  else {
    V_LiftDebounceTimer = 0;
  }
  
  return(L_criteria_met);
}


/******************************************************************************
 * Function:     Lift_Control_Dictator
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/
T_Lift_State Lift_Control_Dictator(bool                L_driver_auto_climb_button,
                                   bool                L_driver_auto_climb_pause,
                                   T_LiftCmndDirection L_DriverLiftCmndDirection,
                                   double              L_game_time,
                                   T_Lift_State        L_current_state,                                
                                   double              L_lift_measured_position_YD,
                                   double              L_lift_measured_position_XD,
                                   double             *L_lift_command_YD,
                                   double             *L_lift_command_XD,
                                   double             *L_Lift_CommandPwr_YD,
                                   double             *L_Lift_CommandPwr_XD,
                                   bool                L_YD_LimitDetected,
                                   bool                L_XD_LimitDetected,
                                   double              L_gyro_yawangledegrees,
                                   double              L_liftMotorYD_CurrentOut,
                                   double              L_liftMotorXD_CurrentOut,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD)
  {
  T_Lift_State L_Commanded_State = L_current_state;
  double L_lift_command_YD_Temp = 0;
  double L_lift_command_XD_Temp = 0;
  double L_lift_command_rate_YD = KV_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1];
  double L_lift_command_rate_XD = KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1];;
  double L_LiftYD_Power = 0;
  double L_LiftXD_Power = 0;

  if (V_LiftXY_Test == true)
    {
    /* Only used for testing. */
    L_lift_command_YD_Temp = V_LiftYD_TestLocation;
    L_lift_command_XD_Temp = V_LiftXD_TestLocation;
    }
  else if (V_LiftInitialized == false)
    {
    if (L_YD_LimitDetected == false)
      {
      L_LiftYD_Power = K_lift_autoResetDown_YD;
      }

    if (L_XD_LimitDetected == false)
      {
      L_LiftXD_Power = K_lift_driver_manual_back_XD;
      }
    
    if (L_YD_LimitDetected == true && 
        L_XD_LimitDetected == true)
      {
      L_LiftYD_Power = 0;
      L_LiftXD_Power = 0;
      V_LiftInitialized = true;

      EncodersLiftInit(m_encoderLiftYD,
                       m_encoderLiftXD);
      }
    }
  else if ((L_driver_auto_climb_pause == true) && (V_Lift_Paused == false))
    {
    /* The driver pressed a button to puase the climb process.  Let's save the current locations and hold. */
    V_Lift_Paused = true;
    V_Lift_PausedXD_Position = L_lift_measured_position_XD;
    V_Lift_PausedYD_Position = L_lift_measured_position_YD;
    /* Set commanded location to current measured location for this loop. */
    L_lift_command_XD_Temp = L_lift_measured_position_XD;
    L_lift_command_YD_Temp = L_lift_measured_position_YD;
    }
  else if (((L_driver_auto_climb_button == true) && (V_Lift_Paused == true)) || 
            (V_Lift_Paused == false))
    {
    V_Lift_Paused = false;
    V_Lift_PausedXD_Position = L_lift_measured_position_XD;
    V_Lift_PausedYD_Position = L_lift_measured_position_YD;

    switch (L_current_state)
      {
        case E_S0_BEGONE:
            if (L_DriverLiftCmndDirection == E_LiftCmndUp)
              {
              L_lift_command_YD_Temp = *L_lift_command_YD + K_lift_driver_up_rate_YD;
              }
            else if (L_DriverLiftCmndDirection == E_LiftCmndDown)
              {
              L_lift_command_YD_Temp = *L_lift_command_YD - K_lift_driver_down_rate_YD;
              }
              else 
              {
                L_lift_command_YD_Temp = *L_lift_command_YD;
              }
            /* The driver should only initiate the state machine once the robot has become suspended. */
            if (L_driver_auto_climb_button == true && L_lift_measured_position_YD >= K_lift_enable_auto_YD) {
                L_Commanded_State = E_S2_lift_down_YD;
            }
        break;

        case E_S2_lift_down_YD:
            V_criteria_met = S2_lift_down_YD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S3_move_forward_XD;
            }
        break;

        case E_S3_move_forward_XD:
            V_criteria_met = S3_move_forward_XD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S4_stretch_up_YD;
            }
        break;

        case E_S4_stretch_up_YD:
            V_criteria_met = S4_stretch_up_YD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S5_more_forward_XD;
            }
        break;

        case E_S5_more_forward_XD:
            V_criteria_met = S5_more_forward_XD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S6_lift_up_more_YD;
            }
        break;

        case E_S6_lift_up_more_YD:
            V_criteria_met = S6_lift_up_more_YD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S7_move_back_XD;
            }
        break;

        case E_S7_move_back_XD:
            V_criteria_met = S7_move_back_XD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, L_gyro_yawangledegrees, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S8_more_down_some_YD;
            }
        break;

        case E_S8_more_down_some_YD:
            V_criteria_met = S8_more_down_some_YD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S9_back_rest_XD;
            }
        break;

        case E_S9_back_rest_XD:
            V_criteria_met = S9_back_rest_XD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State =   E_S10_final_YD;
            }
        break;

        case E_S10_final_YD:
            V_criteria_met = S10_final_YD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true){
              L_Commanded_State = E_S11_final_OWO;
            }
        break;

        case E_S11_final_OWO:
            V_criteria_met = S11_final_OWO(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
            if(V_criteria_met == true && V_LiftIteration < E_LiftIteration2){
              L_Commanded_State = E_S2_lift_down_YD;
              V_LiftIteration = E_LiftIteration2;
            }
            else if(V_criteria_met == true && V_LiftIteration >= E_LiftIteration2){
              L_Commanded_State = E_S11_final_OWO;
            }
        break;
      }
    }
  else
    {
    /* Lift is currently paused: */
    L_lift_command_XD_Temp = V_Lift_PausedXD_Position;
    L_lift_command_YD_Temp = V_Lift_PausedYD_Position;
    }

  /* Place limits on the travel of XD and YD to prevent damage: */
  if (L_lift_command_YD_Temp > K_lift_max_YD)
    {
    L_lift_command_YD_Temp = K_lift_max_YD;
    }
  else if (L_lift_command_YD_Temp < K_lift_min_YD)
    {
    L_lift_command_YD_Temp = K_lift_max_YD;
    }

  if (L_lift_command_XD_Temp > K_lift_max_XD)
    {
    L_lift_command_XD_Temp = K_lift_max_XD;
    }
  else if (L_lift_command_XD_Temp < K_lift_min_XD)
    {
    L_lift_command_XD_Temp = K_lift_max_XD;
    }

  *L_lift_command_YD= RampTo(L_lift_command_YD_Temp, *L_lift_command_YD, L_lift_command_rate_YD);

  *L_lift_command_XD= RampTo(L_lift_command_XD_Temp, *L_lift_command_XD, L_lift_command_rate_XD);

  *L_Lift_CommandPwr_YD = L_LiftYD_Power;
  
  *L_Lift_CommandPwr_XD = L_LiftXD_Power;

  RecordLiftMotorMaxCurrent(L_current_state,
                            L_liftMotorYD_CurrentOut,
                            L_liftMotorXD_CurrentOut);

  return(L_Commanded_State);
}