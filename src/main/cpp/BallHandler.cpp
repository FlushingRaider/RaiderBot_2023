/*
  BallHandler.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to processing of balls, cargo, etc.
  This can include but is not limited to:
   - Intake
   - Elevator
   - Launcher
   - Targeting
 */

#include <math.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "Const.hpp"
#include "Lookup.hpp"

double V_IntakePowerCmnd   = 0;
double V_ElevatorPowerCmnd = 0;
double V_ShooterRPM_Cmnd   = 0;
double V_ShooterTestSpeed  = 0;
bool   V_ShooterTargetSpeedReached = false;
double V_ShooterRPM_CmndPrev = 0;
bool   V_BH_LauncherActive = false; // Indicates when the launcher is being controlled.  When false, motors should be in 0 power command
double KV_ShooterRampRate = 0;
// double V_LauncherPID_Gx[E_PID_SparkMaxCalSz];

#ifdef BallHandlerTest
bool V_BallHandlerTest = true;
double V_LauncherPID_Gx[E_PID_SparkMaxCalSz];
#else
bool V_BallHandlerTest = false;
#endif


/******************************************************************************
 * Function:     BallHandlerMotorConfigs
 *
 * Description:  Contains the motor configurations for the ball handler.
 *               - Intake (power cmnd only)
 *               - Elevator (power cmnd only)
 *               - Launcher (PID Control in motor controller)
 ******************************************************************************/
void BallHandlerMotorConfigsInit(rev::SparkMaxPIDController m_rightShooterpid,
                                 rev::SparkMaxPIDController m_leftShooterpid)
  {
  // set PID coefficients
  # ifdef CompBot
  m_rightShooterpid.SetP(K_BH_LauncherPID_Gx[E_kP]);
  m_rightShooterpid.SetI(K_BH_LauncherPID_Gx[E_kI]);
  m_rightShooterpid.SetD(K_BH_LauncherPID_Gx[E_kD]);
  m_rightShooterpid.SetIZone(K_BH_LauncherPID_Gx[E_kIz]);
  m_rightShooterpid.SetFF(K_BH_LauncherPID_Gx[E_kFF]);
  m_rightShooterpid.SetOutputRange(K_BH_LauncherPID_Gx[E_kMinOutput], K_BH_LauncherPID_Gx[E_kMaxOutput]);

  m_leftShooterpid.SetP(K_BH_LauncherPID_Gx[E_kP]);
  m_leftShooterpid.SetI(K_BH_LauncherPID_Gx[E_kI]);
  m_leftShooterpid.SetD(K_BH_LauncherPID_Gx[E_kD]);
  m_leftShooterpid.SetIZone(K_BH_LauncherPID_Gx[E_kIz]);
  m_leftShooterpid.SetFF(K_BH_LauncherPID_Gx[E_kFF]);
  m_leftShooterpid.SetOutputRange(K_BH_LauncherPID_Gx[E_kMinOutput], K_BH_LauncherPID_Gx[E_kMaxOutput]);
  #endif

  KV_ShooterRampRate = K_BH_LauncherPID_Gx[E_kMaxAcc];

  #ifdef BallHandlerTest
  T_PID_SparkMaxCal L_Index = E_kP;

  for (L_Index = E_kP;
       L_Index < E_PID_SparkMaxCalSz;
       L_Index = T_PID_SparkMaxCal(int(L_Index) + 1))
      {
      V_LauncherPID_Gx[L_Index] = K_BH_LauncherPID_Gx[L_Index];
      }

  // KV_ShooterRampRate = V_LauncherPID_Gx[E_kMaxAcc];
  
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", K_BH_LauncherPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain", K_BH_LauncherPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain", K_BH_LauncherPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone", K_BH_LauncherPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Feed Forward", K_BH_LauncherPID_Gx[E_kFF]);
  frc::SmartDashboard::PutNumber("Max Output", K_BH_LauncherPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output", K_BH_LauncherPID_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("Max Velocity", K_BH_LauncherPID_Gx[E_kMaxVel]);
  frc::SmartDashboard::PutNumber("Min Velocity", K_BH_LauncherPID_Gx[E_kMinVel]);
  frc::SmartDashboard::PutNumber("Max Acceleration", K_BH_LauncherPID_Gx[E_kMaxAcc]);
  frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", K_BH_LauncherPID_Gx[E_kAllErr]);

  frc::SmartDashboard::PutNumber("Launch Speed Desired", V_ShooterTestSpeed);
  #endif
  }


/******************************************************************************
 * Function:     BallHandlerMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the ball handler.
 *               - Intake (power cmnd only)
 *               - Elevator (power cmnd only)
 *               - Launcher (PID Control in motor controller)
 ******************************************************************************/
void BallHandlerMotorConfigsCal(rev::SparkMaxPIDController m_rightShooterpid,
                                rev::SparkMaxPIDController m_leftShooterpid)
  {
  // read PID coefficients from SmartDashboard
  #ifdef BallHandlerTest
  double L_p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double L_i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double L_d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double L_iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double L_max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double L_min = frc::SmartDashboard::GetNumber("Min Output", 0);
  double L_maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
  double L_minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
  double L_maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
  double L_allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

  V_ShooterTestSpeed = frc::SmartDashboard::GetNumber("Launch Speed Desired", 0);

  if((L_p != V_LauncherPID_Gx[E_kP]))   { m_rightShooterpid.SetP(L_p); m_leftShooterpid.SetP(L_p); V_LauncherPID_Gx[E_kP] = L_p; }
  if((L_i != V_LauncherPID_Gx[E_kI]))   { m_rightShooterpid.SetI(L_i); m_leftShooterpid.SetI(L_i); V_LauncherPID_Gx[E_kI] = L_i; }
  if((L_d != V_LauncherPID_Gx[E_kD]))   { m_rightShooterpid.SetD(L_d); m_leftShooterpid.SetD(L_d); V_LauncherPID_Gx[E_kD] = L_d; }
  if((L_iz != V_LauncherPID_Gx[E_kIz])) { m_rightShooterpid.SetIZone(L_iz); m_leftShooterpid.SetIZone(L_iz); V_LauncherPID_Gx[E_kIz] = L_iz; }
  if((L_ff != V_LauncherPID_Gx[E_kFF])) { m_rightShooterpid.SetFF(L_ff); m_leftShooterpid.SetFF(L_ff); V_LauncherPID_Gx[E_kFF] = L_ff; }
  if((L_max != V_LauncherPID_Gx[E_kMaxOutput]) || (L_min != K_BH_LauncherPID_Gx[E_kMinOutput])) { m_rightShooterpid.SetOutputRange(L_min, L_max); m_leftShooterpid.SetOutputRange(L_min, L_max); V_LauncherPID_Gx[E_kMinOutput] = L_min; V_LauncherPID_Gx[E_kMaxOutput] = L_max; }

  // if((L_maxV != V_LauncherPID_Gx[E_kMaxVel])) { m_rightShooterpid.SetSmartMotionMaxVelocity(L_maxV); m_leftShooterpid.SetSmartMotionMaxVelocity(L_maxV); V_LauncherPID_Gx[E_kMaxVel] = L_maxV; }
  // if((L_minV != V_LauncherPID_Gx[E_kMinVel])) { m_rightShooterpid.SetSmartMotionMinOutputVelocity(L_minV); m_leftShooterpid.SetSmartMotionMinOutputVelocity(L_minV); V_LauncherPID_Gx[E_kMinVel] = L_minV; }
  if((L_maxA != V_LauncherPID_Gx[E_kMaxAcc])) { KV_ShooterRampRate = L_maxA; V_LauncherPID_Gx[E_kMaxAcc] = L_maxA; }
  // if((L_allE != V_LauncherPID_Gx[E_kAllErr])) { m_rightShooterpid.SetSmartMotionAllowedClosedLoopError(L_allE); m_leftShooterpid.SetSmartMotionAllowedClosedLoopError(L_allE); V_LauncherPID_Gx[E_kAllErr] = L_allE; }
  #endif
  }


/******************************************************************************
 * Function:     BallHandlerInit
 *
 * Description:  Initialization function for the drive control.
 ******************************************************************************/
void BallHandlerInit()
  {
  V_IntakePowerCmnd = 0;
  V_ElevatorPowerCmnd = 0;
  V_ShooterRPM_Cmnd = 0;
  V_ShooterTargetSpeedReached = false;
  V_ShooterRPM_CmndPrev = 0;
  V_BH_LauncherActive = false;
  }


/******************************************************************************
 * Function:     BallLauncher
 *
 * Description:  Contains the functionality for controlling the launch 
 *               mechanism.
 ******************************************************************************/
double BallLauncher(bool                 L_DisableShooter,
                    bool                 L_AutoShootReq,
                    T_ADAS_ActiveFeature L_ADAS_ActiveFeature,
                    double               L_ADAS_RPM_BH_Launcher,
                    double               L_ManualShooter,
                    double               L_LauncherCurrentSpeed)
  {
  double           L_ShooterSpeedCmnd       = 0;
  double           L_ShooterSpeedCmndTarget = 0;
  T_LauncherStates L_LauncherState          = E_LauncherNotActive;

  if (V_BallHandlerTest == true)
    {
    // This is only used when in test mode
    L_ShooterSpeedCmndTarget = V_ShooterTestSpeed;
    L_LauncherState = E_LauncherManualActive;
    }
  else if (L_ADAS_ActiveFeature > E_ADAS_Disabled)
    {
    /* ADAS is active, pass through the request: */
    L_ShooterSpeedCmndTarget = L_ADAS_RPM_BH_Launcher;
    L_LauncherState = E_LauncherAutoTargetActive;
    }
  else if (fabs(L_ManualShooter) >= K_BH_LauncherManualDb)
    {
    if (L_ManualShooter < 0)
      {
      L_ShooterSpeedCmndTarget = K_BH_LauncherManualHi;
      }
    else
      {
      L_ShooterSpeedCmndTarget = K_BH_LauncherManualLo;
      }
    L_LauncherState = E_LauncherManualActive;
    }

  L_ShooterSpeedCmnd = RampTo(L_ShooterSpeedCmndTarget, V_ShooterRPM_CmndPrev, KV_ShooterRampRate);

  V_ShooterRPM_CmndPrev = L_ShooterSpeedCmnd;
  
  if (fabs(L_ShooterSpeedCmndTarget - L_LauncherCurrentSpeed) <= K_BH_LauncherSpeedDb)
    {
    V_ShooterTargetSpeedReached = true;
    }
  else
    {
    V_ShooterTargetSpeedReached = false;
    }

  /* Determine when to transition into 0 power command: */
  if (fabs(L_ShooterSpeedCmnd) >= K_BH_LauncherMinCmndSpd)
    {
    V_BH_LauncherActive = true;
    }
  else
    {
    V_BH_LauncherActive = false;
    }

  return (L_ShooterSpeedCmnd);
  }

/******************************************************************************
 * Function:     BallIntake
 *
 * Description:  Contains the functionality for controlling the intake 
 *               mechanism.
 ******************************************************************************/
double BallIntake(bool                 L_DriverIntakeInCmnd,
                  bool                 L_DriverIntakeOutCmnd,
                  T_ADAS_ActiveFeature L_ADAS_ActiveFeature,
                  double               L_ADAS_Pct_BH_Intake,
                  bool                 L_LowerBallDetected,
                  bool                 L_UpperBallDetected)
  {
  double L_IntakeMotorCmnd = 0;

  if (L_ADAS_ActiveFeature > E_ADAS_Disabled)
    {
    if ((L_ADAS_Pct_BH_Intake > 0) && 
        ((L_LowerBallDetected == true && L_UpperBallDetected == false) ||
         (L_LowerBallDetected == false && L_UpperBallDetected == true) ||
         (L_UpperBallDetected == false)))
         {
         L_IntakeMotorCmnd = L_ADAS_Pct_BH_Intake;
         }
    }
  else if ((L_DriverIntakeInCmnd == true) && 
           ((L_LowerBallDetected == true && L_UpperBallDetected == false) ||
            (L_LowerBallDetected == false && L_UpperBallDetected == true) ||
            (L_UpperBallDetected == false)))
    {
    L_IntakeMotorCmnd = K_BH_IntakePower;
    }
  else if (L_DriverIntakeOutCmnd == true)
    {
    L_IntakeMotorCmnd = -K_BH_IntakePower;
    }
  // Otherwise, leave at 0

    return (L_IntakeMotorCmnd);
  }

/******************************************************************************
 * Function:     BallElevator
 *
 * Description:  Contains the functionality for controlling the elevator 
 *               mechanism.
 *
 ******************************************************************************/
double BallElevator(bool L_BallDetected,
                    bool L_BallDetectedLower,
                    bool L_ElevatorCmndUp,
                    bool L_ElevatorCmndDwn,
                    bool L_LauncherTargetSpeedReached,
                    T_ADAS_ActiveFeature L_ADAS_ActiveFeature,
                    bool L_ADAS_Pct_BH_Elevator,
                    bool L_Intake,
                    double L_LauncherRPM_Cmnd)
  {
    double L_ElevatorPowerCmnd = 0;

    if(((L_ElevatorCmndUp == true) || 
        (L_Intake == true)) ||

       ((L_ADAS_ActiveFeature > E_ADAS_Disabled) &&
        (L_ADAS_Pct_BH_Elevator > 0)))
      {
        if ((L_BallDetected == false) ||
            ((L_LauncherTargetSpeedReached == true) && (fabs(L_LauncherRPM_Cmnd) > K_BH_LauncherMinCmndSpd)))
          {
          if (L_ADAS_ActiveFeature == E_ADAS_Disabled)
            {
            L_ElevatorPowerCmnd = K_BH_ElevatorPowerUp;
            }
          else
            {
            /* ADAS is active and we are meeting the criteria, elevate da ball */
            L_ElevatorPowerCmnd = L_ADAS_Pct_BH_Elevator;
            }
          }
      }
    else if(L_ElevatorCmndDwn == true)
      {
      L_ElevatorPowerCmnd = K_BH_ElevatorPowerDwn;
      }

    // otherwise leave at 0

    return (L_ElevatorPowerCmnd);
  }


/******************************************************************************
 * Function:     BallHandlerControlMain
 *
 * Description:  Contains the functionality for controlling the launch 
 *               mechanism.
 ******************************************************************************/
void BallHandlerControlMain(bool L_IntakeInCmnd,
                            bool L_IntakeOutCmnd,
                            bool L_BallDetected,
                            bool L_BallDetectedLower,
                            bool L_ElevatorCmndUp,
                            bool L_ElevatorCmndDwn,
                            bool L_DisableShooter,
                            bool L_AutoShootReq,
                            double L_LauncherCurrentSpeed,
                            double L_ManualShooter,
                            T_ADAS_ActiveFeature L_ADAS_ActiveFeature,
                            double L_ADAS_RPM_BH_Launcher,
                            double L_ADAS_Pct_BH_Intake,
                            double L_ADAS_Pct_BH_Elevator,
                            double *L_Intake,
                            double *L_Elevator,
                            double *L_Shooter)
  {
    double L_LauncherRPM       = 0;
    double L_IntakePowerCmnd   = 0;
    double L_ElevatorPowerCmnd = 0;

    L_LauncherRPM = BallLauncher( L_DisableShooter,
                                  L_AutoShootReq,
                                  L_ADAS_ActiveFeature,
                                  L_ADAS_RPM_BH_Launcher,
                                  L_ManualShooter,
                                  L_LauncherCurrentSpeed);

    L_IntakePowerCmnd = BallIntake(L_IntakeInCmnd,
                                   L_IntakeOutCmnd,
                                   L_ADAS_ActiveFeature,
                                   L_ADAS_Pct_BH_Intake,
                                   L_BallDetectedLower,
                                   L_BallDetected);
    
    L_ElevatorPowerCmnd = BallElevator(L_BallDetected,
                                       L_BallDetectedLower,
                                       L_ElevatorCmndUp,
                                       L_ElevatorCmndDwn,
                                       V_ShooterTargetSpeedReached,
                                       L_ADAS_ActiveFeature,
                                       L_ADAS_Pct_BH_Elevator,
                                       L_IntakeInCmnd,
                                       L_LauncherRPM);

    *L_Intake = L_IntakePowerCmnd;

    *L_Elevator = L_ElevatorPowerCmnd;

    *L_Shooter = L_LauncherRPM;
  }