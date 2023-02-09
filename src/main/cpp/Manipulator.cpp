/*
  Manipulator.cpp

   Created on: Feb 04, 2023
   Author: Lauren and Chloe

   This will control the manipulator, aka arm, and its motors, in order to recieve, move, and score game pieces

   lift go brrrrrrrrrrrrrrrrrrr -chloe
   gaslight gatekeep manipulate
 */

#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"

T_Man_State VeMAN_Cnt_Man_state = E_S0_BEGONE; 
T_Man_Iteration VeMAN_Cnt_ManIteration = VeMAN_Cnt_ManIterationNew;

double VeMAN_Cnt_LayoverTimer = 0; // owo, because Chloe
bool   VeMAN_b_CriteriaMet = false;
bool   VeMAN_b_ArmInitialized = false;

double VeMan_Cnt_MoterCommandA = 0; //temp motor variable (temp)
double VeMAN_Cnt_MoterCommandB = 0; //temp motor variable (temp)

double VeMAN_Cnt_MoterTestLocationA = 0; //location it want go (temp)
double VeMAN_Cnt_MoterTestLocationB = 0; //location it want go (temp)

double VeMAN_Cnt_MoterTestPowerCmndA = 0; //power to motor (temp)
double VeMAN_Cnt_MoterTestPowerCmndB = 0; //power to motor (temp)

double VaMAN_v_MotorMaxCurrentA[E_Lift_State_Sz]; //max current(temp)
double VaMAN_v_MotorMaxCurrentB[E_Lift_State_Sz]; //max current(temp)

bool   VeMAN_b_WaitingForDriverINS = false;  // Instrumentation only, but indication that we are waiting for the driver to press button for next step.
bool   VeMAN_b_Paused = false; //Checks to see if paused (for testing)
double VeMAN_b_PausedMoterPositionA = 0; //keeps current value of motor postion when paised (temp)
double VeMAN_b_PausedMoterPositionB = 0; //keeps current value of motor postion when paised (temp)

double VaMAN_InS_RampRateMoterA[E_Lift_State_Sz][E_LiftIterationSz]; //motor ramp rate (temp)
double VaMAN_InS_RampRateMoterB[E_Lift_State_Sz][E_LiftIterationSz]; //motor ramp rate (temp)

#ifdef LiftXY_Test
bool   VeMAN_b_MoterTestA = false; // temporary, we don't want to use the manual overrides
double VMAN_PID_Gx[E_PID_SparkMaxCalSz];
#else
bool VeMAN_b_MoterTestA = false;
#endif


/******************************************************************************
 * Function:     ManipulatorMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the Arm and intake motors.
 *               - A through G
 ******************************************************************************/
void ManipulatorMoterConfigsInit(rev::SparkMaxPIDController m_liftpidYD,
                          rev::SparkMaxPIDController m_liftpidXD)
  {
  T_Man_State     LeMAN_Cnt_Index1;
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

  for (LeMAN_Cnt_Index2 = VeMAN_Cnt_ManIterationNew;
       LeMAN_Cnt_Index2 < E_LiftIterationSz;
       LeMAN_Cnt_Index2 = T_Man_Iteration(int(LeMAN_Cnt_Index2) + 1))
      {
      for (LeMAN_Cnt_Index1 = E_S0_BEGONE;
           LeMAN_Cnt_Index1 < E_Lift_State_Sz;
           LeMAN_Cnt_Index1 = T_Man_State(int(LeMAN_Cnt_Index1) + 1))
          {
          VaMAN_InS_RampRateMoterA[LeMAN_Cnt_Index1][LeMAN_Cnt_Index2] = K_LiftRampRateYD[LeMAN_Cnt_Index1][LeMAN_Cnt_Index2];
          VaMAN_InS_RampRateMoterB[LeMAN_Cnt_Index1][LeMAN_Cnt_Index2] = K_LiftRampRateXD[LeMAN_Cnt_Index1][LeMAN_Cnt_Index2];
          }
      }
  
  #ifdef LiftXY_Test
  T_PID_SparkMaxCal LeMAN_e_Index = E_kP;

  for (LeMAN_e_Index = E_kP;
       LeMAN_e_Index < E_PID_SparkMaxCalSz;
       LeMAN_e_Index = T_PID_SparkMaxCal(int(LeMAN_e_Index) + 1))
      {
      VMAN_PID_Gx[LeMAN_e_Index] = K_LiftPID_Gx[LeMAN_e_Index];
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

  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]",            K_LiftRampRateXD[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]",     K_LiftRampRateXD[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateXD[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
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

  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]",            K_LiftRampRateYD[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]",     K_LiftRampRateYD[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]",   K_LiftRampRateYD[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
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
 * Function:     ManipulatorMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the lift motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
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

  // VeMAN_Cnt_MoterTestLocationA = frc::SmartDashboard::GetNumber("Set Position Y", 0);
  // VeMAN_Cnt_MoterTestLocationB = frc::SmartDashboard::GetNumber("Set Position X", 0);

  // if((L_p != VMAN_PID_Gx[E_kP]))   { m_liftpidYD.SetP(L_p); m_liftpidXD.SetP(L_p); VMAN_PID_Gx[E_kP] = L_p; }
  // if((L_i != VMAN_PID_Gx[E_kI]))   { m_liftpidYD.SetI(L_i); m_liftpidXD.SetI(L_i); VMAN_PID_Gx[E_kI] = L_i; }
  // if((L_d != VMAN_PID_Gx[E_kD]))   { m_liftpidYD.SetD(L_d); m_liftpidXD.SetD(L_d); VMAN_PID_Gx[E_kD] = L_d; }
  // if((L_iz != VMAN_PID_Gx[E_kIz])) { m_liftpidYD.SetIZone(L_iz); m_liftpidXD.SetIZone(L_iz); VMAN_PID_Gx[E_kIz] = L_iz; }
  // if((L_ff != VMAN_PID_Gx[E_kFF])) { m_liftpidYD.SetFF(L_ff); m_liftpidXD.SetFF(L_ff); VMAN_PID_Gx[E_kFF] = L_ff; }
  // if((L_max != VMAN_PID_Gx[E_kMaxOutput]) || (L_min != VMAN_PID_Gx[E_kMinOutput])) { m_liftpidYD.SetOutputRange(L_min, L_max); m_liftpidXD.SetOutputRange(L_min, L_max); VMAN_PID_Gx[E_kMinOutput] = L_min; VMAN_PID_Gx[E_kMaxOutput] = L_max; }
  
  VaMAN_InS_RampRateMoterB[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]",            VaMAN_InS_RampRateMoterB[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterB[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterB[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]",     VaMAN_InS_RampRateMoterB[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterB[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterB[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterB[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         VaMAN_InS_RampRateMoterB[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        VaMAN_InS_RampRateMoterB[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterB[E_S0_BEGONE][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration2]",            VaMAN_InS_RampRateMoterB[E_S0_BEGONE][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S2_lift_down_YD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterB[E_S2_lift_down_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S3_move_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S3_move_forward_XD][E_LiftIteration2]",   VaMAN_InS_RampRateMoterB[E_S3_move_forward_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S4_stretch_up_YD][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S4_stretch_up_YD][E_LiftIteration2]",     VaMAN_InS_RampRateMoterB[E_S4_stretch_up_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S5_more_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S5_more_forward_XD][E_LiftIteration2]",   VaMAN_InS_RampRateMoterB[E_S5_more_forward_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S6_lift_up_more_YD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S6_lift_up_more_YD][E_LiftIteration2]",   VaMAN_InS_RampRateMoterB[E_S6_lift_up_more_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S7_move_back_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S7_move_back_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterB[E_S7_move_back_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S8_more_down_some_YD][E_LiftIteration2]", VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S9_back_rest_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S10_final_YD][E_LiftIteration2]",         VaMAN_InS_RampRateMoterB[E_S10_final_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterB[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S11_final_OWO][E_LiftIteration2]",        VaMAN_InS_RampRateMoterB[E_S11_final_OWO][E_LiftIteration2]);

  VaMAN_InS_RampRateMoterA[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]",            VaMAN_InS_RampRateMoterA[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterA[E_S2_lift_down_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterA[E_S3_move_forward_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]",     VaMAN_InS_RampRateMoterA[E_S4_stretch_up_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterA[E_S5_more_forward_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]",   VaMAN_InS_RampRateMoterA[E_S6_lift_up_more_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterA[E_S7_move_back_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]", VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]",      VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]",         VaMAN_InS_RampRateMoterA[E_S10_final_YD][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]",        VaMAN_InS_RampRateMoterA[E_S11_final_OWO][VeMAN_Cnt_ManIterationNew]);
  VaMAN_InS_RampRateMoterA[E_S0_BEGONE][E_LiftIteration2]            = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration2]",            VaMAN_InS_RampRateMoterA[E_S0_BEGONE][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S2_lift_down_YD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S2_lift_down_YD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterA[E_S2_lift_down_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S3_move_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S3_move_forward_XD][E_LiftIteration2]",   VaMAN_InS_RampRateMoterA[E_S3_move_forward_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S4_stretch_up_YD][E_LiftIteration2]     = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S4_stretch_up_YD][E_LiftIteration2]",     VaMAN_InS_RampRateMoterA[E_S4_stretch_up_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S5_more_forward_XD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S5_more_forward_XD][E_LiftIteration2]",   VaMAN_InS_RampRateMoterA[E_S5_more_forward_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S6_lift_up_more_YD][E_LiftIteration2]   = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S6_lift_up_more_YD][E_LiftIteration2]",   VaMAN_InS_RampRateMoterA[E_S6_lift_up_more_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S7_move_back_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S7_move_back_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterA[E_S7_move_back_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][E_LiftIteration2] = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S8_more_down_some_YD][E_LiftIteration2]", VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][E_LiftIteration2]      = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S9_back_rest_XD][E_LiftIteration2]",      VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S10_final_YD][E_LiftIteration2]         = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S10_final_YD][E_LiftIteration2]",         VaMAN_InS_RampRateMoterA[E_S10_final_YD][E_LiftIteration2]);
  VaMAN_InS_RampRateMoterA[E_S11_final_OWO][E_LiftIteration2]        = frc::SmartDashboard::GetNumber("K_LiftRampRateYD[E_S11_final_OWO][E_LiftIteration2]",        VaMAN_InS_RampRateMoterA[E_S11_final_OWO][E_LiftIteration2]);
  #endif
  }

/******************************************************************************
 * Function:     ManipulatorControlInit
 *
 * Description:  Initialization function for the Manipulator controls.
 ******************************************************************************/
void ManipulatorControlInit()
  {
  T_Man_State LeMAN_e_Index;

  VeMAN_Cnt_Man_state = E_S0_BEGONE;
  VeMAN_Cnt_ManIteration = VeMAN_Cnt_ManIterationNew;
  VeMAN_Cnt_LayoverTimer = 0;
  VeMAN_b_CriteriaMet = false;

  VeMan_Cnt_MoterCommandA = 0;
  VeMAN_Cnt_MoterCommandB = 0;

  VeMAN_Cnt_MoterTestLocationA = 0;
  VeMAN_Cnt_MoterTestLocationB = 0;

  VeMAN_Cnt_MoterTestPowerCmndA = 0;
  VeMAN_Cnt_MoterTestPowerCmndB = 0;

  VeMAN_b_Paused = false;
  VeMAN_b_PausedMoterPositionA = 0;
  VeMAN_b_PausedMoterPositionB = 0;

  VeMAN_b_WaitingForDriverINS = false;

  VeMAN_b_ArmInitialized = false;

  for (LeMAN_e_Index = E_S0_BEGONE;
       LeMAN_e_Index < E_Lift_State_Sz;
       LeMAN_e_Index = T_Man_State(int(LeMAN_e_Index) + 1))
      {
      VaMAN_v_MotorMaxCurrentA[LeMAN_e_Index] = 0;
      VaMAN_v_MotorMaxCurrentB[LeMAN_e_Index] = 0;
      }
  }

/******************************************************************************
 * Function:     RecordManipulatorMotorMaxCurrent
 *
 * Description:  Record the max observed current.  
 *               This is for instrumentation only.
 ******************************************************************************/
void RecordManipulatorMotorMaxCurrent(T_Man_State LeMAN_Cnt_CurrentState,                                
                               double       LeMAN_v_MotorCurrentOutA,
                               double       LeMAN_v_MotorCurrentOutB)
  {
  if (fabs(LeMAN_v_MotorCurrentOutA) > fabs(VaMAN_v_MotorMaxCurrentA[LeMAN_Cnt_CurrentState]))
    {
    VaMAN_v_MotorMaxCurrentA[LeMAN_Cnt_CurrentState] = LeMAN_v_MotorCurrentOutA;
    }
  
  if (fabs(LeMAN_v_MotorCurrentOutB) > fabs(VaMAN_v_MotorMaxCurrentB[LeMAN_Cnt_CurrentState]))
    {
    VaMAN_v_MotorMaxCurrentB[LeMAN_Cnt_CurrentState] = LeMAN_v_MotorCurrentOutB;
    }
  }

/******************************************************************************
 * Function:     Manipulator_Control_ManualOverride
 *
 * Description:  Manual override control used during the FRC test section.
 ******************************************************************************/
void Manipulator_Control_ManualOverride(double *LeMAN_Cmd_CommandA,
                                 double *LeMAN_Cmd_CommandB,
                                 double  LeMAN_v_MotorCurrentOutA,
                                 double  LeMAN_v_MotorCurrentOutB,
                                 T_Manipulator_CmndDirection LeMAN_Cmd_DriverMANDirection,
                                 bool    LeMAN_b_LimitDetectedA,
                                 bool    LeMAN_b_LimitDetectedB)
  {
  double LeMAN_v_MoterPowerA= 0;
  double LeMAN_v_MoterPowerB= 0;
  T_Man_State LeMAN_Cnt_CurrentState = E_S0_BEGONE; // Not really the lift state, but allows us record the max currents

    if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndUp)
      {
      LeMAN_v_MoterPowerA= K_Manipulator_Driver_manual_MoterA;
      LeMAN_Cnt_CurrentState = E_S0_BEGONE;
      }
    else if ((LeMAN_Cmd_DriverMANDirection == E_LiftCmndDown) &&
             (LeMAN_b_LimitDetectedA == false))
      {
      LeMAN_v_MoterPowerA= K_Manipulator_Driver_manual_MoterB;
      LeMAN_Cnt_CurrentState = E_S2_lift_down_YD;
      }
    else if ((LeMAN_Cmd_DriverMANDirection == E_LiftCmndBack) &&
             (LeMAN_b_LimitDetectedB == false))
      {
      LeMAN_v_MoterPowerB= K_lift_driver_manual_back_XD;
      LeMAN_Cnt_CurrentState = E_S7_move_back_XD;
      }
    else if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndForward)
      {
      LeMAN_v_MoterPowerB= K_lift_driver_manual_forward_XD;
      LeMAN_Cnt_CurrentState = E_S3_move_forward_XD;
      }

  RecordManipulatorMotorMaxCurrent(LeMAN_Cnt_CurrentState,                                
                            LeMAN_v_MotorCurrentOutA,
                            LeMAN_v_MotorCurrentOutB);

  *LeMAN_Cmd_CommandA = LeMAN_v_MoterPowerA;
  *LeMAN_Cmd_CommandB = LeMAN_v_MoterPowerB;
  }


/******************************************************************************
 * Function:     S2_lift_down_YD
 *
 * Description:  State 2: moving robert up by moving y-lift down
 ******************************************************************************/
 bool S2_lift_down_YD(double         LeLFT_b_AutoClimbButton,
                      double         LeLFT_In_MeasuredPositionYD,
                      double         LeLFT_In_MeasuredPositionXD,
                      double        *LeMAN_Cmd_CommandA,
                      double        *LeMAN_Cmd_CommandB,
                      double        *LeLFT_InS_CommandRateYD,
                      double        *LeLFT_InS_CommandRateXD,
                      T_Man_Iteration LeLFT_Cmd_LiftIteration)
{
  bool LeLFT_b_CriteriaMet = false;

  *LeMAN_Cmd_CommandA = K_lift_S2_YD;

  *LeMAN_Cmd_CommandB = K_lift_min_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S2_lift_down_YD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S2_lift_down_YD][LeLFT_Cmd_LiftIteration];

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
                         double        *LeMAN_Cmd_CommandA,
                         double        *LeMAN_Cmd_CommandB,
                         double        *LeLFT_InS_CommandRateYD,
                         double        *LeLFT_InS_CommandRateXD,
                         T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeMAN_Cmd_CommandB = K_lift_S3_XD;

  *LeMAN_Cmd_CommandA = K_lift_S3_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S3_move_forward_XD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S3_move_forward_XD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S3_XD + K_lift_deadband_XD) && LeLFT_In_MeasuredPositionXD >= (K_lift_S3_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                       double        *LeMAN_Cmd_CommandA,
                       double        *LeMAN_Cmd_CommandB,
                       double        *LeLFT_InS_CommandRateYD,
                       double        *LeLFT_InS_CommandRateXD,
                       T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
   bool LeLFT_b_CriteriaMet = false;
  
  *LeMAN_Cmd_CommandA = K_lift_S4_YD;

  *LeMAN_Cmd_CommandB = K_lift_S4_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S4_stretch_up_YD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S4_stretch_up_YD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S4_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S4_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
         VeMAN_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                         double        *LeMAN_Cmd_CommandA,
                         double        *LeMAN_Cmd_CommandB,
                         double        *LeLFT_InS_CommandRateYD,
                         double        *LeLFT_InS_CommandRateXD,
                         T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeMAN_Cmd_CommandB = K_lift_S5_XD;

  *LeMAN_Cmd_CommandA = K_lift_S5_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S5_more_forward_XD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S5_more_forward_XD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S5_XD + K_lift_deadband_XD) && LeLFT_In_MeasuredPositionXD >= (K_lift_S5_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                         double        *LeMAN_Cmd_CommandA,
                         double        *LeMAN_Cmd_CommandB,
                         double        *LeLFT_InS_CommandRateYD,
                         double        *LeLFT_InS_CommandRateXD,
                         T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeMAN_Cmd_CommandA = K_lift_S6_YD;

  *LeMAN_Cmd_CommandB = K_lift_S6_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S6_lift_up_more_YD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S6_lift_up_more_YD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S6_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S6_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                      double        *LeMAN_Cmd_CommandA,
                      double        *LeMAN_Cmd_CommandB,
                      double        *LeLFT_InS_CommandRateYD,
                      double        *LeLFT_InS_CommandRateXD,
                      T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;

  *LeMAN_Cmd_CommandB = K_lift_S7_XD;

  *LeMAN_Cmd_CommandA = K_lift_S7_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S7_move_back_XD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S7_move_back_XD][LeLFT_Cmd_LiftIteration]; // Don't go too fast, going slower will help to reduce rocking

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S7_XD + K_lift_deadband_XD)  && LeLFT_In_MeasuredPositionXD >= (K_lift_S7_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
         VeMAN_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                           double        *LeMAN_Cmd_CommandA,
                           double        *LeMAN_Cmd_CommandB,
                           double        *LeLFT_InS_CommandRateYD,
                           double        *LeLFT_InS_CommandRateXD,
                           T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeMAN_Cmd_CommandA = K_lift_S8_YD;

  *LeMAN_Cmd_CommandB = K_lift_S8_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S8_more_down_some_YD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S8_more_down_some_YD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S8_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S8_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
         VeMAN_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                      double        *LeMAN_Cmd_CommandA,
                      double        *LeMAN_Cmd_CommandB,
                      double        *LeLFT_InS_CommandRateYD,
                      double        *LeLFT_InS_CommandRateXD,
                      T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeMAN_Cmd_CommandB = K_lift_S9_XD;

  *LeMAN_Cmd_CommandA = K_lift_S9_YD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S9_back_rest_XD][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S9_back_rest_XD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionXD <= (K_lift_S9_XD + K_lift_deadband_XD) && LeLFT_In_MeasuredPositionXD >= (K_lift_S9_XD - K_lift_deadband_XD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
         VeMAN_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                   double        *LeMAN_Cmd_CommandA,
                   double        *LeMAN_Cmd_CommandB,
                   double        *LeLFT_InS_CommandRateYD,
                   double        *LeLFT_InS_CommandRateXD,
                   T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeMAN_Cmd_CommandA = K_lift_S10_YD;

  *LeMAN_Cmd_CommandB = K_lift_S10_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S10_final_YD][LeLFT_Cmd_LiftIteration]; // Slow down, don't yank too hard

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S10_final_YD][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S10_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S10_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
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
                    double        *LeMAN_Cmd_CommandA,
                    double        *LeMAN_Cmd_CommandB,
                    double        *LeLFT_InS_CommandRateYD,
                    double        *LeLFT_InS_CommandRateXD,
                    T_Man_Iteration LeLFT_Cmd_LiftIteration)  
{
  bool LeLFT_b_CriteriaMet = false;
  
  *LeMAN_Cmd_CommandA = K_lift_S11_YD;

  *LeMAN_Cmd_CommandB = K_lift_S11_XD;

  *LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S11_final_OWO][LeLFT_Cmd_LiftIteration];

  *LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S11_final_OWO][LeLFT_Cmd_LiftIteration];

  if (LeLFT_In_MeasuredPositionYD <= (K_lift_S11_YD + K_lift_deadband_YD) && LeLFT_In_MeasuredPositionYD >= (K_lift_S11_YD - K_lift_deadband_YD)) {
    VeMAN_Cnt_LayoverTimer += C_ExeTime;
    if (VeMAN_Cnt_LayoverTimer >= K_Lift_deadband_timer){
      VeMAN_b_WaitingForDriverINS = true;
      if (LeLFT_b_AutoClimbButton == true){
         /* Let the driver determine when we are not swinging and can proceed */
         LeLFT_b_CriteriaMet = true;
         VeMAN_Cnt_LayoverTimer = 0;
         VeMAN_b_WaitingForDriverINS = false;
      }
    }
  }
  else {
    VeMAN_Cnt_LayoverTimer = 0;
  }
  
  return(LeLFT_b_CriteriaMet);
}


/******************************************************************************
 * Function:     Lift_Control_Dictator
 *
 * Description:  Main calling function for lift control.
 ******************************************************************************/
T_Man_State Lift_Control_Dictator(bool                LeLFT_b_AutoClimbButton,
                                   bool                LeLFT_b_DriverAutoClimbPause,
                                   T_Manipulator_CmndDirection LeMAN_Cmd_DriverMANDirection,
                                   double              LeLFT_SEC_GameTime,
                                   T_Man_State        LeMAN_Cnt_CurrentState,                                
                                   double              LeLFT_In_MeasuredPositionYD,
                                   double              LeLFT_In_MeasuredPositionXD,
                                   double             *LeMAN_Cmd_CommandA,
                                   double             *LeMAN_Cmd_CommandB,
                                   double             *LeLFT_Pct_CommandPwrYD,
                                   double             *LeLFT_Pct_CommandPwrXD,
                                   bool                LeMAN_b_LimitDetectedA,
                                   bool                LeMAN_b_LimitDetectedB,
                                   double              LeLEFT_Deg_GyroAngleYaws,
                                   double              LeMAN_v_MotorCurrentOutA,
                                   double              LeMAN_v_MotorCurrentOutB,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD)
  {
  T_Man_State LeLFT_e_CommandedState = LeMAN_Cnt_CurrentState;
  double LeMAN_Cmd_CommandA_Temp = 0;
  double LeMAN_Cmd_CommandB_Temp = 0;
  double LeLFT_InS_CommandRateYD = VaMAN_InS_RampRateMoterA[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew];
  double LeLFT_InS_CommandRateXD = VaMAN_InS_RampRateMoterB[E_S0_BEGONE][VeMAN_Cnt_ManIterationNew];
  double LeMAN_v_MoterPowerA= 0;
  double LeMAN_v_MoterPowerB= 0;

  if (VeMAN_b_MoterTestA == true)
    {
    /* Only used for testing. */
    LeMAN_Cmd_CommandA_Temp = VeMAN_Cnt_MoterTestLocationA;
    LeMAN_Cmd_CommandB_Temp = VeMAN_Cnt_MoterTestLocationB;
    }
  else if (VeMAN_b_ArmInitialized == false)
    {
    if (LeMAN_b_LimitDetectedA == false)
      {
      LeMAN_v_MoterPowerA= K_lift_autoResetDown_YD;
      }

    if (LeMAN_b_LimitDetectedB == false)
      {
      LeMAN_v_MoterPowerB= K_lift_driver_manual_back_XD;
      }
    
    if (LeMAN_b_LimitDetectedA == true && 
        LeMAN_b_LimitDetectedB == true)
      {
      LeMAN_v_MoterPowerA= 0;
      LeMAN_v_MoterPowerB= 0;
      VeMAN_b_ArmInitialized = true;

      // EncodersLiftInit(m_encoderLiftYD,
      //                  m_encoderLiftXD);
      }
    }
  else if ((LeLFT_b_DriverAutoClimbPause == true) && (VeMAN_b_Paused == false))
    {
    /* The driver pressed a button to puase the climb process.  Let's save the current locations and hold. */
    VeMAN_b_Paused = true;
    VeMAN_b_PausedMoterPositionA = LeLFT_In_MeasuredPositionXD;
    VeMAN_b_PausedMoterPositionB = LeLFT_In_MeasuredPositionYD;
    /* Set commanded location to current measured location for this loop. */
    LeMAN_Cmd_CommandB_Temp = LeLFT_In_MeasuredPositionXD;
    LeMAN_Cmd_CommandA_Temp = LeLFT_In_MeasuredPositionYD;
    }
  else if (((LeLFT_b_AutoClimbButton == true) && (VeMAN_b_Paused == true)) || 
            (VeMAN_b_Paused == false))
    {
    VeMAN_b_Paused = false;
    VeMAN_b_PausedMoterPositionA = LeLFT_In_MeasuredPositionXD;
    VeMAN_b_PausedMoterPositionB = LeLFT_In_MeasuredPositionYD;

    switch (LeMAN_Cnt_CurrentState)
      {
        case E_S0_BEGONE:
            if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndUp)
              {
              LeMAN_Cmd_CommandA_Temp = *LeMAN_Cmd_CommandA + K_lift_driver_up_rate_YD;
              }
            else if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndDown)
              {
              LeMAN_Cmd_CommandA_Temp = *LeMAN_Cmd_CommandA - K_lift_driver_down_rate_YD;
              }
              else 
              {
                LeMAN_Cmd_CommandA_Temp = *LeMAN_Cmd_CommandA;
              }
            /* The driver should only initiate the state machine once the robot has become suspended. */
            if (LeLFT_b_AutoClimbButton == true && LeLFT_In_MeasuredPositionYD >= K_lift_enable_auto_YD) {
                LeLFT_e_CommandedState = E_S2_lift_down_YD;
            }
        break;

        case E_S2_lift_down_YD:
            VeMAN_b_CriteriaMet = S2_lift_down_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S3_move_forward_XD;
            }
        break;

        case E_S3_move_forward_XD:
            VeMAN_b_CriteriaMet = S3_move_forward_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S4_stretch_up_YD;
            }
        break;

        case E_S4_stretch_up_YD:
            VeMAN_b_CriteriaMet = S4_stretch_up_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S5_more_forward_XD;
            }
        break;

        case E_S5_more_forward_XD:
            VeMAN_b_CriteriaMet = S5_more_forward_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S6_lift_up_more_YD;
            }
        break;

        case E_S6_lift_up_more_YD:
            VeMAN_b_CriteriaMet = S6_lift_up_more_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S7_move_back_XD;
            }
        break;

        case E_S7_move_back_XD:
            VeMAN_b_CriteriaMet = S7_move_back_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, LeLEFT_Deg_GyroAngleYaws, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S8_more_down_some_YD;
            }
        break;

        case E_S8_more_down_some_YD:
            VeMAN_b_CriteriaMet = S8_more_down_some_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S9_back_rest_XD;
            }
        break;

        case E_S9_back_rest_XD:
            VeMAN_b_CriteriaMet = S9_back_rest_XD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState =   E_S10_final_YD;
            }
        break;

        case E_S10_final_YD:
            VeMAN_b_CriteriaMet = S10_final_YD(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true){
              LeLFT_e_CommandedState = E_S11_final_OWO;
            }
        break;

        case E_S11_final_OWO:
            VeMAN_b_CriteriaMet = S11_final_OWO(LeLFT_b_AutoClimbButton, LeLFT_In_MeasuredPositionYD, LeLFT_In_MeasuredPositionXD, &LeMAN_Cmd_CommandA_Temp, &LeMAN_Cmd_CommandB_Temp, &LeLFT_InS_CommandRateYD, &LeLFT_InS_CommandRateXD,VeMAN_Cnt_ManIteration);
            if(VeMAN_b_CriteriaMet == true &&VeMAN_Cnt_ManIteration < E_LiftIteration2){
              LeLFT_e_CommandedState = E_S2_lift_down_YD;
             VeMAN_Cnt_ManIteration = E_LiftIteration2;
            }
            else if(VeMAN_b_CriteriaMet == true &&VeMAN_Cnt_ManIteration >= E_LiftIteration2){
              LeLFT_e_CommandedState = E_S11_final_OWO;
            }
        break;
      }
    }
  else
    {
    /* Lift is currently paused: */
    LeMAN_Cmd_CommandB_Temp = VeMAN_b_PausedMoterPositionA;
    LeMAN_Cmd_CommandA_Temp = VeMAN_b_PausedMoterPositionB;
    }

  /* Place limits on the travel of XD and YD to prevent damage: */
  if (LeMAN_Cmd_CommandA_Temp > K_lift_max_YD)
    {
    LeMAN_Cmd_CommandA_Temp = K_lift_max_YD;
    }
  else if (LeMAN_Cmd_CommandA_Temp < K_lift_min_YD)
    {
    LeMAN_Cmd_CommandA_Temp = K_lift_max_YD;
    }

  if (LeMAN_Cmd_CommandB_Temp > K_lift_max_XD)
    {
    LeMAN_Cmd_CommandB_Temp = K_lift_max_XD;
    }
  else if (LeMAN_Cmd_CommandB_Temp < K_lift_min_XD)
    {
    LeMAN_Cmd_CommandB_Temp = K_lift_max_XD;
    }

  *LeMAN_Cmd_CommandA= RampTo(LeMAN_Cmd_CommandA_Temp, *LeMAN_Cmd_CommandA, LeLFT_InS_CommandRateYD);

  *LeMAN_Cmd_CommandB= RampTo(LeMAN_Cmd_CommandB_Temp, *LeMAN_Cmd_CommandB, LeLFT_InS_CommandRateXD);

  *LeLFT_Pct_CommandPwrYD = LeMAN_v_MoterPowerA;
  
  *LeLFT_Pct_CommandPwrXD = LeMAN_v_MoterPowerB;

  RecordManipulatorMotorMaxCurrent(LeMAN_Cnt_CurrentState,
                            LeMAN_v_MotorCurrentOutA,
                            LeMAN_v_MotorCurrentOutB);

  return(LeLFT_e_CommandedState);
}