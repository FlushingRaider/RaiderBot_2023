/*
  BallHandler.hpp

  Created on: Feb 15, 2022
  Author: Biggs

  This header file contains functions related to processing of balls, cargo, etc.
  This can include but is not limited to:
   - Intake
   - Elevator
   - Launcher
   - Targeting
 */

extern double V_IntakePowerCmnd;
extern double V_ElevatorPowerCmnd;
extern double V_ShooterRPM_Cmnd;
extern bool   V_ShooterTargetSpeedReached;
extern bool   V_BH_LauncherActive;

void BallHandlerMotorConfigsInit(rev::SparkMaxPIDController m_rightShooterpid,
                                 rev::SparkMaxPIDController m_leftShooterpid);

void BallHandlerMotorConfigsCal(rev::SparkMaxPIDController m_rightShooterpid,
                                rev::SparkMaxPIDController m_leftShooterpid);

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
                            double *L_Shooter);

                            

void BallHandlerInit(void);
