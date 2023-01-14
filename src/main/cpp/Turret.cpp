/*
  Turret.cpp

  Created on: Aug 15, 2022
  Author: Biggs

  This file contains functions related to control of the turret on the robot.
  This can include but is not limited to:
   - Turret initialization
   - Camera targeting
 */

#include "rev/CANSparkMax.h" // remove
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"

/* V_e_TurretState: Indicates the state of turret control. */
TeTurretState V_e_TurretState = E_TurretDisabled;

/* V_e_TurretInitialization: Indicates the state of turret initizlization. */
TeTurretInitialization V_e_TurretInitialization = E_TurretInitDisabled;

/* V_t_TurretDebounceTimer: Indicates the debounce time of the current state. */
double V_t_TurretDebounceTimer = 0;

/* V_t_TurretTimeOutTimer: Indicates the current time of the state. */
double V_t_TurretStateTimer = 0;

/* V_deg_TurretPositionPrev: Indicates the previous indicated position of the turret. */
double V_deg_TurretPositionPrev = 0;


/******************************************************************************
 * Function:     TurretInitialization
 *
 * Description:  Contains functionality for initailization of the turret.
 ******************************************************************************/
TeTurretInitialization TurretInitialization(TeTurretInitialization   L_e_TurretInitialization,
                                            TsRobotMotorCmnd        *LsRobotMotorCmnd)
  {
    // TeTurretInitialization L_e_TurretInitialization = E_TurretInitDisabled;

    
    return(L_e_TurretInitialization);
  }









// T_Lift_State V_Lift_state = E_S0_BEGONE;
// T_Lift_Iteration V_LiftIteration = E_LiftIteration1;

// double V_LiftDebounceTimer = 0; // owo, because Chloe
// bool   V_criteria_met = false;
// bool   V_LiftInitialized = false;

// double V_lift_command_YD = 0;
// double V_lift_command_XD = 0;

// double V_LiftYD_TestLocation = 0;
// double V_LiftXD_TestLocation = 0;

// double V_LiftYD_TestPowerCmnd = 0;
// double V_LiftXD_TestPowerCmnd = 0;

// double V_LiftMotorYD_MaxCurrent[E_Lift_State_Sz];
// double V_LiftMotorXD_MaxCurrent[E_Lift_State_Sz];

// bool   V_Lift_WaitingForDriverINS = false;  // Instrumentation only, but indication that we are waiting for the driver to press button for next step.
// bool   V_Lift_Paused = false;
// double V_Lift_PausedXD_Position = 0;
// double V_Lift_PausedYD_Position = 0;

// double KV_LiftRampRateYD[E_Lift_State_Sz][E_LiftIterationSz];
// double KV_LiftRampRateXD[E_Lift_State_Sz][E_LiftIterationSz];

// #ifdef LiftXY_Test
// bool   V_LiftXY_Test = false; // temporary, we don't want to use the manual overrides
// double V_LiftPID_Gx[E_PID_SparkMaxCalSz];
// #else
// bool V_LiftXY_Test = false;
// #endif


/******************************************************************************
 * Function:     TurretMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the lift motors.
 *               - XD and YD
 ******************************************************************************/
void TurretMotorConfigsInit(WPI_TalonSRX m_turretMotor)
  {
  // Configure motor and set PID coefficients
  m_turretMotor.SetSelectedSensorPosition(0);
  m_turretMotor.ConfigFactoryDefault();
  m_turretMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, K_t_TurretTimeoutMs);
  m_turretMotor.SetSensorPhase(true);
  m_turretMotor.SetSelectedSensorPosition(0);
  m_turretMotor.ConfigNominalOutputForward(0, K_t_TurretTimeoutMs);
  m_turretMotor.ConfigNominalOutputReverse(0, K_t_TurretTimeoutMs);
  m_turretMotor.ConfigPeakOutputForward(1, K_t_TurretTimeoutMs);
  m_turretMotor.ConfigPeakOutputReverse(-1, K_t_TurretTimeoutMs);
  m_turretMotor.Config_kD(1, 0.1);
  m_turretMotor.Config_kP(1, 0.1);
  m_turretMotor.Config_kI(1, 0.0001);

  #ifdef Turret_Test
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
  #endif
  }


// /******************************************************************************
//  * Function:     LiftMotorConfigsCal
//  *
//  * Description:  Contains the motor configurations for the lift motors.  This 
//  *               allows for rapid calibration, but must not be used for comp.
//  ******************************************************************************/
// void TurretMotorConfigsCal(WPI_TalonSRX m_turretMotor)
//   {
//   // read PID coefficients from SmartDashboard
//   #ifdef LiftXY_Test
//   // double L_p = frc::SmartDashboard::GetNumber("P Gain", 0);
//   // double L_i = frc::SmartDashboard::GetNumber("I Gain", 0);
//   // double L_d = frc::SmartDashboard::GetNumber("D Gain", 0);
//   // double L_iz = frc::SmartDashboard::GetNumber("I Zone", 0);
//   // double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
//   // double L_max = frc::SmartDashboard::GetNumber("Max Output", 0);
//   // double L_min = frc::SmartDashboard::GetNumber("Min Output", 0);
//   // double L_maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
//   // double L_minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
//   // double L_maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
//   // double L_allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

//   // V_LiftYD_TestLocation = frc::SmartDashboard::GetNumber("Set Position Y", 0);
//   // V_LiftXD_TestLocation = frc::SmartDashboard::GetNumber("Set Position X", 0);

//   // if((L_p != V_LiftPID_Gx[E_kP]))   { m_liftpidYD.SetP(L_p); m_liftpidXD.SetP(L_p); V_LiftPID_Gx[E_kP] = L_p; }
//   // if((L_i != V_LiftPID_Gx[E_kI]))   { m_liftpidYD.SetI(L_i); m_liftpidXD.SetI(L_i); V_LiftPID_Gx[E_kI] = L_i; }
//   // if((L_d != V_LiftPID_Gx[E_kD]))   { m_liftpidYD.SetD(L_d); m_liftpidXD.SetD(L_d); V_LiftPID_Gx[E_kD] = L_d; }
//   // if((L_iz != V_LiftPID_Gx[E_kIz])) { m_liftpidYD.SetIZone(L_iz); m_liftpidXD.SetIZone(L_iz); V_LiftPID_Gx[E_kIz] = L_iz; }
//   // if((L_ff != V_LiftPID_Gx[E_kFF])) { m_liftpidYD.SetFF(L_ff); m_liftpidXD.SetFF(L_ff); V_LiftPID_Gx[E_kFF] = L_ff; }
//   // if((L_max != V_LiftPID_Gx[E_kMaxOutput]) || (L_min != V_LiftPID_Gx[E_kMinOutput])) { m_liftpidYD.SetOutputRange(L_min, L_max); m_liftpidXD.SetOutputRange(L_min, L_max); V_LiftPID_Gx[E_kMinOutput] = L_min; V_LiftPID_Gx[E_kMaxOutput] = L_max; }
  
//   KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]",            KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1]);
//   KV_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]",      KV_LiftRampRateXD[E_S2_lift_down_YD][E_LiftIteration1]);
//   #endif
//   }

// /******************************************************************************
//  * Function:     TurretControlInit
//  *
//  * Description:  Initialization function for the turret control.
//  ******************************************************************************/
// void TurretControlInit()
//   {
//   V_e_TurretInitialization = E_TurretInitDisabled;
//   }

// /******************************************************************************
//  * Function:     Lift_Control_ManualOverride
//  *
//  * Description:  Manual override control used during the FRC test section.
//  ******************************************************************************/
// void Lift_Control_ManualOverride(double *L_lift_command_YD,
//                                  double *L_lift_command_XD,
//                                  double  L_liftMotorYD_CurrentOut,
//                                  double  L_liftMotorXD_CurrentOut,
//                                  T_LiftCmndDirection L_DriverLiftCmndDirection,
//                                  bool    L_YD_LimitDetected,
//                                  bool    L_XD_LimitDetected)
//   {
//   double L_LiftYD_Power = 0;
//   double L_LiftXD_Power = 0;
//   T_Lift_State L_current_state = E_S0_BEGONE; // Not really the lift state, but allows us record the max currents

//     if (L_DriverLiftCmndDirection == E_LiftCmndUp)
//       {
//       L_LiftYD_Power = K_lift_driver_manual_up_YD;
//       L_current_state = E_S0_BEGONE;
//       }
//     else if ((L_DriverLiftCmndDirection == E_LiftCmndDown) &&
//              (L_YD_LimitDetected == false))
//       {
//       L_LiftYD_Power = K_lift_driver_manual_down_YD;
//       L_current_state = E_S2_lift_down_YD;
//       }
//     else if ((L_DriverLiftCmndDirection == E_LiftCmndBack) &&
//              (L_XD_LimitDetected == false))
//       {
//       L_LiftXD_Power = K_lift_driver_manual_back_XD;
//       L_current_state = E_S7_move_back_XD;
//       }
//     else if (L_DriverLiftCmndDirection == E_LiftCmndForward)
//       {
//       L_LiftXD_Power = K_lift_driver_manual_forward_XD;
//       L_current_state = E_S3_move_forward_XD;
//       }

//   *L_lift_command_YD = L_LiftYD_Power;
//   *L_lift_command_XD = L_LiftXD_Power;
//   }


// /******************************************************************************
//  * Function:     S2_lift_down_YD
//  *
//  * Description:  State 2: moving robert up by moving y-lift down
//  ******************************************************************************/
//  bool S2_lift_down_YD(double         L_driver_auto_climb_button,
//                       double         L_lift_measured_position_YD,
//                       double         L_lift_measured_position_XD,
//                       double        *L_lift_command_YD,
//                       double        *L_lift_command_XD,
//                       double        *L_lift_command_rate_YD,
//                       double        *L_lift_command_rate_XD,
//                       T_Lift_Iteration L_LiftIteration)
// {
//   bool L_criteria_met = false;

//   *L_lift_command_YD = K_lift_S2_YD;

//   *L_lift_command_XD = K_lift_min_XD;

//   *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S2_lift_down_YD][L_LiftIteration];

//   *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S2_lift_down_YD][L_LiftIteration];

//   if (L_lift_measured_position_YD <= (K_lift_S2_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S2_YD - K_lift_deadband_YD)) {
//     L_criteria_met = true;
//   }

//   return(L_criteria_met);
// }

// /******************************************************************************
//  * Function:       TurretInitOL
//  *
//  * Description:  Open loop control of turret, looking for limit switch
//  ******************************************************************************/
//  TeTurretInitialization TurretInitOL(double                  L_deg_Encoder,
//                                      bool                    L_b_LimitSwitch,
//                                      bool                    L_b_EncoderReset,
//                                      TeTurretInitialization  L_e_TurretInitState,
//                                      T_MotorControlType     *L_e_MotorControlType,
//                                      double                 *L_pct_TurretMotorCmnd)
// {
//   double L_pct_MotorCmnd = 0.0;
//   double L_deg_DeltaPosition = 0.0;
//   double L_pct_TurretMotorCmndPrev = *L_pct_TurretMotorCmnd;

//   L_deg_DeltaPosition = L_deg_Encoder - V_deg_TurretPositionPrev;

//   V_t_TurretStateTimer += C_ExeTime;

//   // Check to see if we are moving along:
//   if (L_e_TurretInitState == E_TurretInitOL_Right)
//     {
//       if (L_deg_DeltaPosition < K_deg_TurretMinDeltaOL)
//         {
//           V_t_TurretDebounceTimer += C_ExeTime;
//         }
//     }
//   else if (L_e_TurretInitState == E_TurretInitOL_Left)
//     {
//       if (L_deg_DeltaPosition < (-K_deg_TurretMinDeltaOL))
//         {
//           V_t_TurretDebounceTimer += C_ExeTime;
//         }
//     }
  
//   // Let's check what we should be commanding:
//   if (L_e_TurretInitState == E_TurretInitDisabled)
//     {
//       L_e_TurretInitState     = E_TurretInitOL_Right;
//       L_pct_MotorCmnd         = K_Pct_TurretOpenLoopCmnd;
//       V_t_TurretDebounceTimer = 0.0;
//     }
//   else if (L_b_LimitSwitch == true)
//     {
//       // We seem to have hit the limit switch.  Let's zero the encoder, stop the motor and advance to the next phase
//       L_e_TurretInitState     = E_TurretInitLimitSwitch;
//       L_pct_MotorCmnd         = 0.0;
//       V_t_TurretDebounceTimer = 0.0;
//     }
//   else if (V_t_TurretStateTimer > K_t_TurretOL_Timeout)
//     {
//       L_e_TurretInitState = E_TurretInitFaultDetected;
//       L_pct_MotorCmnd         = 0.0;
//       V_t_TurretDebounceTimer = 0.0;
//       V_t_TurretStateTimer    = 0.0;
//     }
//   else if (V_t_TurretDebounceTimer >= K_t_TurretDebounceTimeout)
//     {
//       if (L_e_TurretInitState == E_TurretInitOL_Right)
//         {
//           L_e_TurretInitState     = E_TurretInitOL_Left;
//           L_pct_MotorCmnd         = -K_Pct_TurretOpenLoopCmnd;
//           V_t_TurretDebounceTimer = 0.0;
//         }
//       else // (L_e_TurretInitState == E_TurretInitOL_Left)
//         {
//           // Well, we seem to have hit something else again, let's not break anything and stop.
//           L_e_TurretInitState = E_TurretInitFaultDetected;
//           L_pct_MotorCmnd         = 0.0;
//           V_t_TurretDebounceTimer = 0.0;
//         }
//     }
//   else
//     {
//       L_pct_MotorCmnd = *L_pct_TurretMotorCmnd;
//     }
  
//   V_deg_TurretPositionPrev = L_deg_Encoder;
  
//   *L_e_MotorControlType  = E_MotorControlPctCmnd;
//   *L_pct_TurretMotorCmnd = L_pct_MotorCmnd;

//   return(L_e_TurretInitState);
// }


// /******************************************************************************
//  * Function:       TurretInitLimitSwReturnToZero
//  *
//  * Description:  Open loop control of turret, looking for limit switch
//  ******************************************************************************/
//  TeTurretInitialization TurretInitLimitSwReturnToZero(double                  L_deg_Encoder,
//                                                       bool                    L_b_LimitSwitch,
//                                                       TeTurretInitialization  L_e_TurretInitState,
//                                                       T_MotorControlType     *L_e_MotorControlType,
//                                                       T_MotorControlType     *L_e_MotorControlType,
//                                                       double                 *L_deg_TurretMotorCmnd)
// {
//   double L_pct_MotorCmnd = 0.0;
//   double L_deg_DeltaPosition = 0.0;
//   double L_deg_TurretMotorCmndPrev = *L_deg_TurretMotorCmnd;

//   L_deg_DeltaPosition = L_deg_Encoder - V_deg_TurretPositionPrev;

//   V_t_TurretStateTimer += C_ExeTime;

//   // Check to see if we are moving along:
//   if (L_e_TurretInitState == E_TurretInitOL_Right)
//     {
//       if (L_deg_DeltaPosition < K_deg_TurretMinDeltaOL)
//         {
//           V_t_TurretDebounceTimer += C_ExeTime;
//         }
//     }
//   else if (L_e_TurretInitState == E_TurretInitOL_Left)
//     {
//       if (L_deg_DeltaPosition < (-K_deg_TurretMinDeltaOL))
//         {
//           V_t_TurretDebounceTimer += C_ExeTime;
//         }
//     }
  
//   // Let's check what we should be commanding:
//   if (L_e_TurretInitState == E_TurretInitDisabled)
//     {
//       L_e_TurretInitState     = E_TurretInitOL_Right;
//       L_pct_MotorCmnd         = K_Pct_TurretOpenLoopCmnd;
//       V_t_TurretDebounceTimer = 0.0;
//     }
//   else if (L_b_LimitSwitch == true)
//     {
//       // We seem to have hit the limit switch.  Let's zero the encoder, stop the motor and advance to the next phase
//       L_e_TurretInitState     = E_TurretInitLimitSwitch;
//       L_pct_MotorCmnd         = 0.0;
//       V_t_TurretDebounceTimer = 0.0;
//     }
//   else if (V_t_TurretStateTimer > K_t_TurretOL_Timeout)
//     {
//       L_e_TurretInitState = E_TurretInitFaultDetected;
//       L_pct_MotorCmnd         = 0.0;
//       V_t_TurretDebounceTimer = 0.0;
//       V_t_TurretStateTimer    = 0.0;
//     }
//   else if (V_t_TurretDebounceTimer >= K_t_TurretDebounceTimeout)
//     {
//       if (L_e_TurretInitState == E_TurretInitOL_Right)
//         {
//           L_e_TurretInitState     = E_TurretInitOL_Left;
//           L_pct_MotorCmnd         = -K_Pct_TurretOpenLoopCmnd;
//           V_t_TurretDebounceTimer = 0.0;
//         }
//       else // (L_e_TurretInitState == E_TurretInitOL_Left)
//         {
//           // Well, we seem to have hit something else again, let's not break anything and stop.
//           L_e_TurretInitState = E_TurretInitFaultDetected;
//           L_pct_MotorCmnd         = 0.0;
//           V_t_TurretDebounceTimer = 0.0;
//         }
//     }
//   else
//     {
//       L_pct_MotorCmnd = *L_pct_TurretMotorCmnd;
//     }
  
//   V_deg_TurretPositionPrev = L_deg_Encoder;

//   *L_e_MotorControlType  = E_MotorControlPosition;
  
//   *L_pct_TurretMotorCmnd = L_pct_MotorCmnd;

//   return(L_e_TurretInitState);
// }


// /******************************************************************************
//  * Function:       S4_stretch_up_YD,
//  *
//  * Description:  State 4: x lift no move, y lift go
//  ******************************************************************************/
//  bool S4_stretch_up_YD(double         L_driver_auto_climb_button,
//                        double         L_lift_measured_position_YD,
//                        double         L_lift_measured_position_XD,
//                        double        *L_lift_command_YD,
//                        double        *L_lift_command_XD,
//                        double        *L_lift_command_rate_YD,
//                        double        *L_lift_command_rate_XD,
//                        T_Lift_Iteration L_LiftIteration)  
// {
//    bool L_criteria_met = false;
  
//   *L_lift_command_YD = K_lift_S4_YD;

//   *L_lift_command_XD = K_lift_S4_XD;

//   *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S4_stretch_up_YD][L_LiftIteration];

//   *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S4_stretch_up_YD][L_LiftIteration];

//   if (L_lift_measured_position_YD <= (K_lift_S4_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S4_YD - K_lift_deadband_YD)) {
//     V_LiftDebounceTimer += C_ExeTime;
//     if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
//       V_Lift_WaitingForDriverINS = true;
//       if (L_driver_auto_climb_button == true){
//          /* Let the driver determine when we are not swinging and can proceed */
//          L_criteria_met = true;
//          V_LiftDebounceTimer = 0;
//          V_Lift_WaitingForDriverINS = false;
//       }
//     }
//   }
//   else {
//     V_LiftDebounceTimer = 0;
//   }
  
//   return(L_criteria_met);
// }


// /******************************************************************************
//  * Function:       S11_final_OWO
//  *
//  * Description:  State 11: uwu
//  ******************************************************************************/
//  bool S11_final_OWO(double         L_driver_auto_climb_button,
//                     double         L_lift_measured_position_YD,
//                     double         L_lift_measured_position_XD,
//                     double        *L_lift_command_YD,
//                     double        *L_lift_command_XD,
//                     double        *L_lift_command_rate_YD,
//                     double        *L_lift_command_rate_XD,
//                     T_Lift_Iteration L_LiftIteration)  
// {
//   bool L_criteria_met = false;
  
//   *L_lift_command_YD = K_lift_S11_YD;

//   *L_lift_command_XD = K_lift_S11_XD;

//   *L_lift_command_rate_YD = KV_LiftRampRateYD[E_S11_final_OWO][L_LiftIteration];

//   *L_lift_command_rate_XD = KV_LiftRampRateXD[E_S11_final_OWO][L_LiftIteration];

//   if (L_lift_measured_position_YD <= (K_lift_S11_YD + K_lift_deadband_YD) && L_lift_measured_position_YD >= (K_lift_S11_YD - K_lift_deadband_YD)) {
//     V_LiftDebounceTimer += C_ExeTime;
//     if (V_LiftDebounceTimer >= K_Lift_deadband_timer){
//       V_Lift_WaitingForDriverINS = true;
//       if (L_driver_auto_climb_button == true){
//          /* Let the driver determine when we are not swinging and can proceed */
//          L_criteria_met = true;
//          V_LiftDebounceTimer = 0;
//          V_Lift_WaitingForDriverINS = false;
//       }
//     }
//   }
//   else {
//     V_LiftDebounceTimer = 0;
//   }
  
//   return(L_criteria_met);
// }


























// /******************************************************************************
//  * Function:     TurretControlMain
//  *
//  * Description:  Main calling function for turret control.
//  ******************************************************************************/
// void TurretControlMain(bool                L_driver_auto_climb_button,
//                                    bool                L_driver_auto_climb_pause,
//                                    T_LiftCmndDirection L_DriverLiftCmndDirection,
//                                    double              L_game_time,
//                                    T_Lift_State        L_current_state,                                
//                                    double              L_lift_measured_position_YD,
//                                    double              L_lift_measured_position_XD,
//                                    double             *L_lift_command_YD,
//                                    double             *L_lift_command_XD,
//                                    double             *L_Lift_CommandPwr_YD,
//                                    double             *L_Lift_CommandPwr_XD,
//                                    bool                L_YD_LimitDetected,
//                                    bool                L_XD_LimitDetected,
//                                    double              L_gyro_yawangledegrees,
//                                    double              L_liftMotorYD_CurrentOut,

//                                    bool                L_b_LimitSwitch,
//                                    double              L_deg_Encoder,
//                                    TsRobotMotorCmnd  *LsRobotMotorCmnd)
//   {
//   double L_pct_TurretMotorCmnd = 0.0;
//   bool   L_b_TurretEncoderReset = false;

//   V_e_TurretInitialization = TurretInitialization(V_e_TurretInitialization,
//                                                   LsRobotMotorCmnd);

//   T_Lift_State L_Commanded_State = L_current_state;
//   double L_lift_command_YD_Temp = 0;
//   double L_lift_command_XD_Temp = 0;
//   double L_lift_command_rate_YD = KV_LiftRampRateYD[E_S0_BEGONE][E_LiftIteration1];
//   double L_lift_command_rate_XD = KV_LiftRampRateXD[E_S0_BEGONE][E_LiftIteration1];;
//   double L_LiftYD_Power = 0;
//   double L_LiftXD_Power = 0;

//   if (V_LiftXY_Test == true)
//     {
//     /* Only used for testing. */
//     L_lift_command_YD_Temp = V_LiftYD_TestLocation;
//     L_lift_command_XD_Temp = V_LiftXD_TestLocation;
//     }
//   else if (V_e_TurretInitialization < E_TurretInitComplete)
//     {
//   ,
//   ,
//   ,
//   ,
//   ,
//   E_TurretInitComplete,
//   E_TurretInitFaultDetected
//     switch (V_e_TurretInitialization)
//       {
//         case E_TurretInitDisabled:
//         case E_TurretInitOL_Right:
//         case E_TurretInitOL_Left:
//             V_e_TurretInitialization = TurretInitOL( L_deg_Encoder,
//                                                      L_b_LimitSwitch,
//                                                      V_e_TurretInitialization,
//                                                     *L_pct_TurretMotorCmnd)
//         break;

//         case E_TurretInitLimitSwitch:
//             L_b_TurretEncoderReset = true;
//         break;
//         case E_TurretInitRotateToZeroPosition:
//             V_e_TurretInitialization = TurretInitOL( L_deg_Encoder,
//                                                      L_b_LimitSwitch,
//                                                      V_e_TurretInitialization,
//                                                     *L_pct_TurretMotorCmnd)
//         break;

//       }


//     if (L_YD_LimitDetected == false)
//       {
//       L_LiftYD_Power = K_lift_autoResetDown_YD;
//       }

//     if (L_XD_LimitDetected == false)
//       {
//       L_LiftXD_Power = K_lift_driver_manual_back_XD;
//       }
    
//     if (L_YD_LimitDetected == true && 
//         L_XD_LimitDetected == true)
//       {
//       L_LiftYD_Power = 0;
//       L_LiftXD_Power = 0;
//       V_LiftInitialized = true;

//       EncodersLiftInit(m_encoderLiftYD,
//                        m_encoderLiftXD);
//       }
//     }
//   else if ((L_driver_auto_climb_pause == true) && (V_Lift_Paused == false))
//     {
//     /* The driver pressed a button to puase the climb process.  Let's save the current locations and hold. */
//     V_Lift_Paused = true;
//     V_Lift_PausedXD_Position = L_lift_measured_position_XD;
//     V_Lift_PausedYD_Position = L_lift_measured_position_YD;
//     /* Set commanded location to current measured location for this loop. */
//     L_lift_command_XD_Temp = L_lift_measured_position_XD;
//     L_lift_command_YD_Temp = L_lift_measured_position_YD;
//     }
//   else if (((L_driver_auto_climb_button == true) && (V_Lift_Paused == true)) || 
//             (V_Lift_Paused == false))
//     {
//     V_Lift_Paused = false;
//     V_Lift_PausedXD_Position = L_lift_measured_position_XD;
//     V_Lift_PausedYD_Position = L_lift_measured_position_YD;

//     switch (L_current_state)
//       {
//         case E_S0_BEGONE:
//             if (L_DriverLiftCmndDirection == E_LiftCmndUp)
//               {
//               L_lift_command_YD_Temp = *L_lift_command_YD + K_lift_driver_up_rate_YD;
//               }
//             else if (L_DriverLiftCmndDirection == E_LiftCmndDown)
//               {
//               L_lift_command_YD_Temp = *L_lift_command_YD - K_lift_driver_down_rate_YD;
//               }
//               else 
//               {
//                 L_lift_command_YD_Temp = *L_lift_command_YD;
//               }
//             /* The driver should only initiate the state machine once the robot has become suspended. */
//             if (L_driver_auto_climb_button == true && L_lift_measured_position_YD >= K_lift_enable_auto_YD) {
//                 L_Commanded_State = E_S2_lift_down_YD;
//             }
//         break;

//         case E_S2_lift_down_YD:
//             V_criteria_met = S2_lift_down_YD(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
//             if(V_criteria_met == true){
//               L_Commanded_State =   E_S3_move_forward_XD;
//             }
//         break;

//         case E_S11_final_OWO:
//             V_criteria_met = S11_final_OWO(L_driver_auto_climb_button, L_lift_measured_position_YD, L_lift_measured_position_XD, &L_lift_command_YD_Temp, &L_lift_command_XD_Temp, &L_lift_command_rate_YD, &L_lift_command_rate_XD, V_LiftIteration);
//             if(V_criteria_met == true && V_LiftIteration < E_LiftIteration2){
//               L_Commanded_State = E_S2_lift_down_YD;
//               V_LiftIteration = E_LiftIteration2;
//             }
//             else if(V_criteria_met == true && V_LiftIteration >= E_LiftIteration2){
//               L_Commanded_State = E_S11_final_OWO;
//             }
//         break;
//       }
//     }
//   else
//     {
//     /* Lift is currently paused: */
//     L_lift_command_XD_Temp = V_Lift_PausedXD_Position;
//     L_lift_command_YD_Temp = V_Lift_PausedYD_Position;
//     }

//   /* Place limits on the travel of XD and YD to prevent damage: */
//   if (L_lift_command_YD_Temp > K_lift_max_YD)
//     {
//     L_lift_command_YD_Temp = K_lift_max_YD;
//     }
//   else if (L_lift_command_YD_Temp < K_lift_min_YD)
//     {
//     L_lift_command_YD_Temp = K_lift_max_YD;
//     }

//   if (L_lift_command_XD_Temp > K_lift_max_XD)
//     {
//     L_lift_command_XD_Temp = K_lift_max_XD;
//     }
//   else if (L_lift_command_XD_Temp < K_lift_min_XD)
//     {
//     L_lift_command_XD_Temp = K_lift_max_XD;
//     }

//   *L_lift_command_YD= RampTo(L_lift_command_YD_Temp, *L_lift_command_YD, L_lift_command_rate_YD);

//   *L_lift_command_XD= RampTo(L_lift_command_XD_Temp, *L_lift_command_XD, L_lift_command_rate_XD);

//   *L_Lift_CommandPwr_YD = L_LiftYD_Power;
  
//   *L_Lift_CommandPwr_XD = L_LiftXD_Power;

//   LsRobotMotorCmnd->deg_TurretCmnd = L_pct_TurretMotorCmnd;

//   return(L_Commanded_State);
// }