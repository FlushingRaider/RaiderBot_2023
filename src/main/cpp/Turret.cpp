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









// T_Lift_State VeMAN_CnT_Man_DoesStuffMaybe = E_S0_Rest;
// T_Man_IterationVeLFT_Cnt_LiftIteration = VeLFT_Cnt_LiftIteration1;

// double VeMAN_Cnt_LayoverTimer = 0; // owo, because Chloe
// bool   VeMAN_b_CriteriaMet = false;
// bool   VeMAN_b_ArmInitialized = false;

// double VeMan_Cnt_MoterCommandTurret = 0;
// double VeMan_Cnt_MoterCommandArmPivot = 0;

// double VeMAN_Cnt_MoterTestLocationTurret = 0;
// double VeMAN_Cnt_MoterTestLocationArmPivot = 0;

// double VeMAN_Cnt_MoterTestPowerCmndTurret = 0;
// double VeMAN_Cnt_MoterTestPowerCmndArmPivot = 0;

// double VaMAN_v_MotorMaxCurrentTurret[E_Man_State_Sz];
// double VaMAN_v_MotorMaxCurrentArmPivot[E_Man_State_Sz];

// bool   VeMAN_b_WaitingForDriverINS = false;  // Instrumentation only, but indication that we are waiting for the driver to press button for next step.
// bool   VeLFT_b_Paused = false;
// double VeMAN_Cnt_MoterTestPowerCmndTurret = 0;
// double VeMAN_Cnt_MoterTestPowerCmndArmPivot = 0;

// double VaMAN_InS_RampRateMoterTurret[E_Man_State_Sz][E_LiftIterationSz];
// double VaMAN_InS_RampRateMoterShoulder[E_Man_State_Sz][E_LiftIterationSz];

// #ifdef LiftXY_Test
// bool   VeMAN_b_MoterTestTurret = false; // temporary, we don't want to use the manual overrides
// double V_LiftPID_Gx[E_PID_SparkMaxCalSz];
// #else
// bool VeMAN_b_MoterTestTurret = false;
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

  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S0_Rest][VeLFT_Cnt_LiftIteration1]",            K_LiftRampRateXD[E_S0_Rest][VeLFT_Cnt_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S1_Intake][VeLFT_Cnt_LiftIteration1]",      K_LiftRampRateXD[E_S1_Intake][VeLFT_Cnt_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S2_TradeOff][VeLFT_Cnt_LiftIteration1]",   K_LiftRampRateXD[E_S2_TradeOff][VeLFT_Cnt_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S3_Swiper][VeLFT_Cnt_LiftIteration1]",     K_LiftRampRateXD[E_S3_Swiper][VeLFT_Cnt_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S4_DrivingState][VeLFT_Cnt_LiftIteration1]",   K_LiftRampRateXD[E_S4_DrivingState][VeLFT_Cnt_LiftIteration1]);
  frc::SmartDashboard::PutNumber("K_LiftRampRateXD[E_S5_Positioning][VeLFT_Cnt_LiftIteration1]",   K_LiftRampRateXD[E_S5_Positioning][VeLFT_Cnt_LiftIteration1]);
  #endif
  }


// /******************************************************************************
//  * Function:     ManipulatorMotorConfigsCal
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

//   // VeMAN_Cnt_MoterTestLocationTurret = frc::SmartDashboard::GetNumber("Set Position Y", 0);
//   // VeMAN_Cnt_MoterTestLocationArmPivot = frc::SmartDashboard::GetNumber("Set Position X", 0);

//   // if((L_p != V_LiftPID_Gx[E_kP]))   { m_liftpidYD.SetP(L_p); m_liftpidXD.SetP(L_p); V_LiftPID_Gx[E_kP] = L_p; }
//   // if((L_i != V_LiftPID_Gx[E_kI]))   { m_liftpidYD.SetI(L_i); m_liftpidXD.SetI(L_i); V_LiftPID_Gx[E_kI] = L_i; }
//   // if((L_d != V_LiftPID_Gx[E_kD]))   { m_liftpidYD.SetD(L_d); m_liftpidXD.SetD(L_d); V_LiftPID_Gx[E_kD] = L_d; }
//   // if((L_iz != V_LiftPID_Gx[E_kIz])) { m_liftpidYD.SetIZone(L_iz); m_liftpidXD.SetIZone(L_iz); V_LiftPID_Gx[E_kIz] = L_iz; }
//   // if((L_ff != V_LiftPID_Gx[E_kFF])) { m_liftpidYD.SetFF(L_ff); m_liftpidXD.SetFF(L_ff); V_LiftPID_Gx[E_kFF] = L_ff; }
//   // if((L_max != V_LiftPID_Gx[E_kMaxOutput]) || (L_min != V_LiftPID_Gx[E_kMinOutput])) { m_liftpidYD.SetOutputRange(L_min, L_max); m_liftpidXD.SetOutputRange(L_min, L_max); V_LiftPID_Gx[E_kMinOutput] = L_min; V_LiftPID_Gx[E_kMaxOutput] = L_max; }
  
//   VaMAN_InS_RampRateMoterShoulder[E_S0_Rest][VeLFT_Cnt_LiftIteration1]            = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S0_Rest][VeLFT_Cnt_LiftIteration1]",            VaMAN_InS_RampRateMoterShoulder[E_S0_Rest][VeLFT_Cnt_LiftIteration1]);
//   VaMAN_InS_RampRateMoterShoulder[E_S1_Intake][VeLFT_Cnt_LiftIteration1]      = frc::SmartDashboard::GetNumber("K_LiftRampRateXD[E_S1_Intake][VeLFT_Cnt_LiftIteration1]",      VaMAN_InS_RampRateMoterShoulder[E_S1_Intake][VeLFT_Cnt_LiftIteration1]);
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
//  * Function:     Manipulator_Control_ManualOverride
//  *
//  * Description:  Manual override control used during the FRC test section.
//  ******************************************************************************/
// void Manipulator_Control_ManualOverride(double *LeMAN_Cmd_CommandTurret,
//                                  double *LeMAN_Cmd_CommandArmPivot,
//                                  double  LeMAN_v_MotorCurrentOutTurret,
//                                  double  LeMAN_v_MotorCurrentOutArmPivot,
//                                  T_Manipulator_CmndDirection LeMAN_Cmd_DriverMANDirection,
//                                  bool    LeMAN_b_LimitDetectedTurret,
//                                  bool    LeMAN_b_LimitDetectedArmPivot)
//   {
//   double LeMAN_v_MoterPowerTurret= 0;
//   double LeMAN_v_MoterPowerArmPivot= 0;
//   T_Lift_State LeMAN_Cnt_CurrentState = E_S0_Rest; // Not really the lift state, but allows us record the max currents

//     if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndUp)
//       {
//       LeMAN_v_MoterPowerTurret= K_Manipulator_Driver_manual_MoterTurret;
//       LeMAN_Cnt_CurrentState = E_S0_Rest;
//       }
//     else if ((LeMAN_Cmd_DriverMANDirection == E_LiftCmndDown) &&
//              (LeMAN_b_LimitDetectedTurret == false))
//       {
//       LeMAN_v_MoterPowerTurret= K_Manipulator_Driver_manual_MoterArmPivot;
//       LeMAN_Cnt_CurrentState = E_S1_Intake;
//       }
//     else if ((LeMAN_Cmd_DriverMANDirection == E_LiftCmndBack) &&
//              (LeMAN_b_LimitDetectedArmPivot == false))
//       {
//       LeMAN_v_MoterPowerArmPivot= K_lift_driver_manual_back_XD;
//       LeMAN_Cnt_CurrentState = E_S6_DroppingTheLoot;
//       }
//     else if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndForward)
//       {
//       LeMAN_v_MoterPowerArmPivot= K_lift_driver_manual_forward_XD;
//       LeMAN_Cnt_CurrentState = E_S2_TradeOff;
//       }

//   *LeMAN_Cmd_CommandTurret = LeMAN_v_MoterPowerTurret;
//   *LeMAN_Cmd_CommandArmPivot = LeMAN_v_MoterPowerArmPivot;
//   }


// /******************************************************************************
//  * Function:     S2_TradeOff
//  *
//  * Description:  State 2: moving robert up by moving y-lift down
//  ******************************************************************************/
//  bool S2_TradeOff(double         LeMAN_b_AutoManipulateButton,
//                       double         LeMAN_Deg_MeasuredAngleTurret,
//                       double         LeMAN_Deg_MeasuredAngleArmPivot,
//                       double        *LeMAN_Cmd_CommandTurret,
//                       double        *LeMAN_Cmd_CommandArmPivot,
//                       double        *LeMAN_InS_CommandRateTurret,
//                       double        *LeMAN_InS_CommandRateArmPivot,
//                       T_Man_Iteration LeMAN_CmdStateIteration)
// {
//   bool LeLFT_b_CriteriaMet = false;

//   *LeMAN_Cmd_CommandTurret = K_lift_S2_YD;

//   *LeMAN_Cmd_CommandArmPivot = K_lift_min_XD;

//   *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S1_Intake][LeMAN_CmdStateIteration];

//   *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S1_Intake][LeMAN_CmdStateIteration];

//   if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S2_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S2_YD - K_lift_deadband_YD)) {
//     LeLFT_b_CriteriaMet = true;
//   }

//   return(LeLFT_b_CriteriaMet);
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
//  * Function:       S4_DrivingState,
//  *
//  * Description:  State 4: x lift no move, y lift go
//  ******************************************************************************/
//  bool S4_DrivingState(double         LeMAN_b_AutoManipulateButton,
//                        double         LeMAN_Deg_MeasuredAngleTurret,
//                        double         LeMAN_Deg_MeasuredAngleArmPivot,
//                        double        *LeMAN_Cmd_CommandTurret,
//                        double        *LeMAN_Cmd_CommandArmPivot,
//                        double        *LeMAN_InS_CommandRateTurret,
//                        double        *LeMAN_InS_CommandRateArmPivot,
//                        T_Man_Iteration LeMAN_CmdStateIteration)  
// {
//    bool LeLFT_b_CriteriaMet = false;
  
//   *LeMAN_Cmd_CommandTurret = K_lift_S4_YD;

//   *LeMAN_Cmd_CommandArmPivot = K_lift_S4_XD;

//   *LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S3_Swiper][LeMAN_CmdStateIteration];

//   *LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S3_Swiper][LeMAN_CmdStateIteration];

//   if (LeMAN_Deg_MeasuredAngleTurret <= (K_lift_S4_YD + K_lift_deadband_YD) && LeMAN_Deg_MeasuredAngleTurret >= (K_lift_S4_YD - K_lift_deadband_YD)) {
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


// /******************************************************************************
//  * Function:       S11_final_OWO
//  *
//  * Description:  State 11: uwu
//  ******************************************************************************/
//  bool S11_final_OWO(double         LeMAN_b_AutoManipulateButton,
//                     double         LeMAN_Deg_MeasuredAngleTurret,
//                     double         LeMAN_Deg_MeasuredAngleArmPivot,
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


























// /******************************************************************************
//  * Function:     TurretControlMain
//  *
//  * Description:  Main calling function for turret control.
//  ******************************************************************************/
// void TurretControlMain(bool                LeMAN_b_AutoManipulateButton,
//                                    bool                L_driver_auto_climb_pause,
//                                    T_Manipulator_CmndDirection LeMAN_Cmd_DriverMANDirection,
//                                    double              L_game_time,
//                                    T_Lift_State        LeMAN_Cnt_CurrentState,                                
//                                    double              LeMAN_Deg_MeasuredAngleTurret,
//                                    double              LeMAN_Deg_MeasuredAngleArmPivot,
//                                    double             *LeMAN_Cmd_CommandTurret,
//                                    double             *LeMAN_Cmd_CommandArmPivot,
//                                    double             *LeLFT_Pct_CommandPwrYD,
//                                    double             *LeLFT_Pct_CommandPwrXD,
//                                    bool                LeMAN_b_LimitDetectedTurret,
//                                    bool                LeMAN_b_LimitDetectedArmPivot,
//                                    double              LeLEFT_Deg_GyroAngleYaws,
//                                    double              LeMAN_v_MotorCurrentOutTurret,

//                                    bool                L_b_LimitSwitch,
//                                    double              L_deg_Encoder,
//                                    TsRobotMotorCmnd  *LsRobotMotorCmnd)
//   {
//   double L_pct_TurretMotorCmnd = 0.0;
//   bool   L_b_TurretEncoderReset = false;

//   V_e_TurretInitialization = TurretInitialization(V_e_TurretInitialization,
//                                                   LsRobotMotorCmnd);

//   T_Lift_State LeLFT_e_CommandedState = LeMAN_Cnt_CurrentState;
//   double LeMAN_Cmd_CommandTurret_Temp = 0;
//   double LeMAN_Cmd_CommandArmPivot_Temp = 0;
//   double LeMAN_InS_CommandRateTurret = VaMAN_InS_RampRateMoterTurret[E_S0_Rest][VeLFT_Cnt_LiftIteration1];
//   double LeMAN_InS_CommandRateArmPivot = VaMAN_InS_RampRateMoterShoulder[E_S0_Rest][VeLFT_Cnt_LiftIteration1];;
//   double LeMAN_v_MoterPowerTurret= 0;
//   double LeMAN_v_MoterPowerArmPivot= 0;

//   if (VeMAN_b_MoterTestTurret == true)
//     {
//     /* Only used for testing. */
//     LeMAN_Cmd_CommandTurret_Temp = VeMAN_Cnt_MoterTestLocationTurret;
//     LeMAN_Cmd_CommandArmPivot_Temp = VeMAN_Cnt_MoterTestLocationArmPivot;
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


//     if (LeMAN_b_LimitDetectedTurret == false)
//       {
//       LeMAN_v_MoterPowerTurret= K_lift_autoResetDown_YD;
//       }

//     if (LeMAN_b_LimitDetectedArmPivot == false)
//       {
//       LeMAN_v_MoterPowerArmPivot= K_lift_driver_manual_back_XD;
//       }
    
//     if (LeMAN_b_LimitDetectedTurret == true && 
//         LeMAN_b_LimitDetectedArmPivot == true)
//       {
//       LeMAN_v_MoterPowerTurret= 0;
//       LeMAN_v_MoterPowerArmPivot= 0;
//       VeMAN_b_ArmInitialized = true;

//       EncodersLiftInit(m_encoderLiftYD,
//                        m_encoderLiftXD);
//       }
//     }
//   else if ((LeLFT_b_DriverAutoClimbPause == true) && (VeLFT_b_Paused == false))
//     {
//     /* The driver pressed a button to puase the climb process.  Let's save the current locations and hold. */
//     VeLFT_b_Paused = true;
//     VeMAN_Cnt_MoterTestPowerCmndTurret = LeMAN_Deg_MeasuredAngleArmPivot;
//     VeMAN_Cnt_MoterTestPowerCmndArmPivot = LeMAN_Deg_MeasuredAngleTurret;
//     /* Set commanded location to current measured location for this loop. */
//     LeMAN_Cmd_CommandArmPivot_Temp = LeMAN_Deg_MeasuredAngleArmPivot;
//     LeMAN_Cmd_CommandTurret_Temp = LeMAN_Deg_MeasuredAngleTurret;
//     }
//   else if (((LeMAN_b_AutoManipulateButton == true) && (VeLFT_b_Paused == true)) || 
//             (VeLFT_b_Paused == false))
//     {
//     VeLFT_b_Paused = false;
//     VeMAN_Cnt_MoterTestPowerCmndTurret = LeMAN_Deg_MeasuredAngleArmPivot;
//     VeMAN_Cnt_MoterTestPowerCmndArmPivot = LeMAN_Deg_MeasuredAngleTurret;

//     switch (LeMAN_Cnt_CurrentState)
//       {
//         case E_S0_Rest:
//             if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndUp)
//               {
//               LeMAN_Cmd_CommandTurret_Temp = *LeMAN_Cmd_CommandTurret + K_lift_driver_up_rate_YD;
//               }
//             else if (LeMAN_Cmd_DriverMANDirection == E_LiftCmndDown)
//               {
//               LeMAN_Cmd_CommandTurret_Temp = *LeMAN_Cmd_CommandTurret - K_lift_driver_down_rate_YD;
//               }
//               else 
//               {
//                 LeMAN_Cmd_CommandTurret_Temp = *LeMAN_Cmd_CommandTurret;
//               }
//             /* The driver should only initiate the state machine once the robot has become suspended. */
//             if (LeMAN_b_AutoManipulateButton == true && LeMAN_Deg_MeasuredAngleTurret >= K_lift_enable_auto_YD) {
//                 LeLFT_e_CommandedState = E_S1_Intake;
//             }
//         break;

//         case E_S1_Intake:
//             VeMAN_b_CriteriaMet = S2_TradeOff(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeLFT_Cnt_LiftIteration);
//             if(VeMAN_b_CriteriaMet == true){
//               LeLFT_e_CommandedState =   E_S2_TradeOff;
//             }
//         break;

//         case E_S11_final_OWO:
//             VeMAN_b_CriteriaMet = S11_final_OWO(LeMAN_b_AutoManipulateButton, LeMAN_Deg_MeasuredAngleTurret, LeMAN_Deg_MeasuredAngleArmPivot, &LeMAN_Cmd_CommandTurret_Temp, &LeMAN_Cmd_CommandArmPivot_Temp, &LeMAN_InS_CommandRateTurret, &LeMAN_InS_CommandRateArmPivot,VeLFT_Cnt_LiftIteration);
//             if(VeMAN_b_CriteriaMet == true &&VeLFT_Cnt_LiftIteration < E_LiftIteration2){
//               LeLFT_e_CommandedState = E_S1_Intake;
//              VeLFT_Cnt_LiftIteration = E_LiftIteration2;
//             }
//             else if(VeMAN_b_CriteriaMet == true &&VeLFT_Cnt_LiftIteration >= E_LiftIteration2){
//               LeLFT_e_CommandedState = E_S11_final_OWO;
//             }
//         break;
//       }
//     }
//   else
//     {
//     /* Lift is currently paused: */
//     LeMAN_Cmd_CommandArmPivot_Temp = VeMAN_Cnt_MoterTestPowerCmndTurret;
//     LeMAN_Cmd_CommandTurret_Temp = VeMAN_Cnt_MoterTestPowerCmndArmPivot;
//     }

//   /* Place limits on the travel of XD and YD to prevent damage: */
//   if (LeMAN_Cmd_CommandTurret_Temp > K_lift_max_YD)
//     {
//     LeMAN_Cmd_CommandTurret_Temp = K_lift_max_YD;
//     }
//   else if (LeMAN_Cmd_CommandTurret_Temp < K_lift_min_YD)
//     {
//     LeMAN_Cmd_CommandTurret_Temp = K_lift_max_YD;
//     }

//   if (LeMAN_Cmd_CommandArmPivot_Temp > K_lift_max_XD)
//     {
//     LeMAN_Cmd_CommandArmPivot_Temp = K_lift_max_XD;
//     }
//   else if (LeMAN_Cmd_CommandArmPivot_Temp < K_lift_min_XD)
//     {
//     LeMAN_Cmd_CommandArmPivot_Temp = K_lift_max_XD;
//     }

//   *LeMAN_Cmd_CommandTurret= RampTo(LeMAN_Cmd_CommandTurret_Temp, *LeMAN_Cmd_CommandTurret, LeMAN_InS_CommandRateTurret);

//   *LeMAN_Cmd_CommandArmPivot= RampTo(LeMAN_Cmd_CommandArmPivot_Temp, *LeMAN_Cmd_CommandArmPivot, LeMAN_InS_CommandRateArmPivot);

//   *LeLFT_Pct_CommandPwrYD = LeMAN_v_MoterPowerTurret;
  
//   *LeLFT_Pct_CommandPwrXD = LeMAN_v_MoterPowerArmPivot;

//   LsRobotMotorCmnd->deg_TurretCmnd = L_pct_TurretMotorCmnd;

//   return(LeLFT_e_CommandedState);
// }