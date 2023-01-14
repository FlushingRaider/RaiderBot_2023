/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561
 */

#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Const.hpp"

double V_WheelAngle[E_RobotCornerSz];
double V_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the NEO and processed to only be from 0 - 360
double V_Deg_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double V_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double V_Deg_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double V_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double V_M_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double V_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts
double V_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts
double V_ShooterSpeedCurr;
double V_Cnt_WheelDeltaDistanceInit[E_RobotCornerSz];
double V_Delta_Angle[E_RobotCornerSz]; // The delta of the angle needed to align the wheels when the robot inits
double V_LiftPostitionYD; // Position of the YD lift, in revolutions of the motor
double V_LiftPostitionXD; // Position of the XD lift, in revolutions of the motor
double V_TurretPosition;


/******************************************************************************
 * Function:     EncodersInitCommon
 *
 * Description:  Initialize all of the applicable encoder variables that are 
 *               common to both bots.
 ******************************************************************************/
void EncodersInitCommon(rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive)
  {
    T_RobotCorner L_Index;

    for (L_Index = E_FrontLeft;
         L_Index < E_RobotCornerSz;
         L_Index = T_RobotCorner(int(L_Index) + 1))
      {
        V_Deg_WheelAngleFwd[L_Index] = 0;
        V_Rad_WheelAngleFwd[L_Index] = 0;
        V_WheelVelocity[L_Index] = 0;
        V_M_WheelDeltaDistance[L_Index] = 0;
        V_Cnt_WheelDeltaDistanceCurr[L_Index] = 0;
        V_Cnt_WheelDeltaDistancePrev[L_Index] = 0;
      }

    m_encoderFrontRightSteer.SetPosition(0);
    m_encoderFrontLeftSteer.SetPosition(0);
    m_encoderRearRightSteer.SetPosition(0);
    m_encoderRearLeftSteer.SetPosition(0);

    m_encoderFrontRightDrive.SetPosition(0);
    m_encoderFrontLeftDrive.SetPosition(0);
    m_encoderRearRightDrive.SetPosition(0);
    m_encoderRearLeftDrive.SetPosition(0);

    V_ShooterSpeedCurr = 0;
  }

/******************************************************************************
 * Function:     EncodersInit
 *
 * Description:  Initialize all of the applicable encoder variables that are 
 *               unique to the comp bot.
 ******************************************************************************/
void EncodersInitComp(rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                      rev::SparkMaxRelativeEncoder m_encoderLiftXD,
                      rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                      rev::SparkMaxRelativeEncoder m_encoderleftShooter)
  {
    m_encoderLiftYD.SetPosition(0);
    m_encoderLiftXD.SetPosition(0);

    m_encoderrightShooter.SetPosition(0);
    m_encoderleftShooter.SetPosition(0);
  }

/******************************************************************************
 * Function:     EncodersLiftInit
 *
 * Description:  Initialize all of the applicable encoder variables.
 ******************************************************************************/
void EncodersLiftInit(rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                      rev::SparkMaxRelativeEncoder m_encoderLiftXD)
  {
    m_encoderLiftYD.SetPosition(0);
    m_encoderLiftXD.SetPosition(0);
  }



/******************************************************************************
 * Function:     EncoderAngleLimit
 *
 * Description:  Limit encoder angle to a given bound.
 ******************************************************************************/
double EncoderAngleLimit(double L_EncoderRawValue,
                         double L_EncoderRawToAngleCal,
                         double L_EncoderOffset,
                         double L_EncoderLimit)
  {
  double L_EncoderConverted = 0;

  L_EncoderConverted = L_EncoderRawValue * L_EncoderRawToAngleCal - L_EncoderLimit + L_EncoderOffset;

  if (L_EncoderConverted > L_EncoderLimit)
    {
      L_EncoderConverted -= (2 * L_EncoderLimit);
    }
  else if (L_EncoderConverted < -L_EncoderLimit)
    {
      L_EncoderConverted += (2 * L_EncoderLimit);
    }
    
  return (L_EncoderConverted);
  }


/******************************************************************************
 * Function:     Read_Encoders
 *
 * Description:  Run all of the encoder decoding logic.
 ******************************************************************************/
void Read_Encoders(double                       L_encoderWheelAngleFrontLeftRaw,
                   double                       L_encoderWheelAngleFrontRightRaw,
                   double                       L_encoderWheelAngleRearLeftRaw,
                   double                       L_encoderWheelAngleRearRightRaw,
                   double                       L_encoderWheelAngleFrontLeftRawPracticeBot,
                   double                       L_encoderWheelAngleFrontRightRawPracticeBot,
                   double                       L_encoderWheelAngleRearLeftRawPracticeBot,
                   double                       L_encoderWheelAngleRearRightRawPracticeBot,
                   rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                   rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                   rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                   rev::SparkMaxRelativeEncoder m_encoderleftShooter,
                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                   rev::SparkMaxRelativeEncoder m_encoderLiftXD,
                   double                       L_encoderTurretAngle)
  {
  T_RobotCorner index;

  V_LiftPostitionYD = m_encoderLiftYD.GetPosition();
  V_LiftPostitionXD = m_encoderLiftXD.GetPosition();

#ifdef CompBot
  V_WheelAngleConverted[E_FrontLeft]  = std::fmod((L_encoderWheelAngleFrontLeftRaw  * C_EncoderToAngle), 360) - K_SD_WheelOffsetAngle[E_FrontLeft];
  V_WheelAngleConverted[E_FrontRight] = std::fmod((L_encoderWheelAngleFrontRightRaw * C_EncoderToAngle), 360) - K_SD_WheelOffsetAngle[E_FrontRight];
  V_WheelAngleConverted[E_RearLeft]   = std::fmod((L_encoderWheelAngleRearLeftRaw   * C_EncoderToAngle), 360) - K_SD_WheelOffsetAngle[E_RearLeft];
  V_WheelAngleConverted[E_RearRight]  = std::fmod((L_encoderWheelAngleRearRightRaw  * C_EncoderToAngle), 360) - K_SD_WheelOffsetAngle[E_RearRight];
#endif
#ifdef PracticeBot
  // V_WheelAngleConverted[E_FrontLeft]  = std::fmod(((L_encoderWheelAngleFrontLeftRawPracticeBot  * C_VoltageToAngle) + K_SD_WheelOffsetAnglePractieBot[E_FrontLeft]), 360) - 180;// std::fmod((L_encoderWheelAngleFrontLeftRawPracticeBot  * C_VoltageToAngle), 360) - K_SD_WheelOffsetAnglePractieBot[E_FrontLeft];
  // V_WheelAngleConverted[E_FrontRight] = std::fmod(((L_encoderWheelAngleFrontRightRawPracticeBot * C_VoltageToAngle) + K_SD_WheelOffsetAnglePractieBot[E_FrontRight]), 360) - 180;
  // V_WheelAngleConverted[E_RearLeft]   = std::fmod(((L_encoderWheelAngleRearLeftRawPracticeBot   * C_VoltageToAngle) + K_SD_WheelOffsetAnglePractieBot[E_RearLeft]) - 180, 180);
  V_WheelAngleConverted[E_FrontLeft] = EncoderAngleLimit(-L_encoderWheelAngleFrontLeftRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_FrontLeft], 180);
  V_WheelAngleConverted[E_FrontRight] = EncoderAngleLimit(-L_encoderWheelAngleFrontRightRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_FrontRight], 180);
  V_WheelAngleConverted[E_RearLeft] = EncoderAngleLimit(-L_encoderWheelAngleRearLeftRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_RearLeft], 180);
  V_WheelAngleConverted[E_RearRight]  = EncoderAngleLimit(-L_encoderWheelAngleRearRightRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_RearRight], 180);
#endif
  V_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();
  
  for (index = E_FrontLeft;
       index < E_RobotCornerSz;
       index = T_RobotCorner(int(index) + 1))
    {
    V_Deg_WheelAngleFwd[index] = V_WheelAngleConverted[index];

    if (V_Deg_WheelAngleFwd[index] > 180)
      {
      V_Deg_WheelAngleFwd[index] -= 360;
      }
    else if (V_Deg_WheelAngleFwd[index] < -180)
      {
      V_Deg_WheelAngleFwd[index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (V_Deg_WheelAngleFwd[index] >= 0)
      {
      V_Deg_WheelAngleRev[index] = V_Deg_WheelAngleFwd[index] - 180;
      }
    else
      {
      V_Deg_WheelAngleRev[index] = V_Deg_WheelAngleFwd[index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    V_Rad_WheelAngleFwd[index] = V_Deg_WheelAngleFwd[index] * (C_PI/180);

    V_M_WheelDeltaDistance[index]  = ((((V_Cnt_WheelDeltaDistanceCurr[index]  - V_Cnt_WheelDeltaDistancePrev[index])/  K_ReductionRatio)) * K_WheelCircufrence );
    V_Cnt_WheelDeltaDistancePrev[index]  = V_Cnt_WheelDeltaDistanceCurr[index];
    }

  V_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / K_ReductionRatio) / 60) * K_WheelCircufrence;

  V_ShooterSpeedCurr = m_encoderrightShooter.GetVelocity(); // We use the right shooter as the reference as this is rotating in the positive direction

  V_TurretPosition = L_encoderTurretAngle * (-K_k_TurretEncoderScaler); // Negative to rotate the output.  Positive is clockwise when viewed from top.
  }


/******************************************************************************
 * Function:     Read_Encoders2
 *
 * Description:  Run all of the encoder decoding logic, for practice bot.
 ******************************************************************************/
void Read_Encoders2(double                       L_encoderWheelAngleFrontLeftRawPracticeBot,
                    double                       L_encoderWheelAngleFrontRightRawPracticeBot,
                    double                       L_encoderWheelAngleRearLeftRawPracticeBot,
                    double                       L_encoderWheelAngleRearRightRawPracticeBot,
                    rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                    rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                    rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                    rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                    double                       L_encoderTurretAngle)
  {
  T_RobotCorner index;

  V_WheelAngleConverted[E_FrontLeft] = EncoderAngleLimit(-L_encoderWheelAngleFrontLeftRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_FrontLeft], 180);
  V_WheelAngleConverted[E_FrontRight] = EncoderAngleLimit(-L_encoderWheelAngleFrontRightRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_FrontRight], 180);
  V_WheelAngleConverted[E_RearLeft] = EncoderAngleLimit(-L_encoderWheelAngleRearLeftRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_RearLeft], 180);
  V_WheelAngleConverted[E_RearRight]  = EncoderAngleLimit(-L_encoderWheelAngleRearRightRawPracticeBot, C_VoltageToAngle, K_SD_WheelOffsetAnglePractieBot[E_RearRight], 180);

  V_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  V_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();
  
  for (index = E_FrontLeft;
       index < E_RobotCornerSz;
       index = T_RobotCorner(int(index) + 1))
    {
    V_Deg_WheelAngleFwd[index] = V_WheelAngleConverted[index];

    if (V_Deg_WheelAngleFwd[index] > 180)
      {
      V_Deg_WheelAngleFwd[index] -= 360;
      }
    else if (V_Deg_WheelAngleFwd[index] < -180)
      {
      V_Deg_WheelAngleFwd[index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (V_Deg_WheelAngleFwd[index] >= 0)
      {
      V_Deg_WheelAngleRev[index] = V_Deg_WheelAngleFwd[index] - 180;
      }
    else
      {
      V_Deg_WheelAngleRev[index] = V_Deg_WheelAngleFwd[index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    V_Rad_WheelAngleFwd[index] = V_Deg_WheelAngleFwd[index] * (C_PI/180);

    V_M_WheelDeltaDistance[index]  = ((((V_Cnt_WheelDeltaDistanceCurr[index]  - V_Cnt_WheelDeltaDistancePrev[index])/  K_ReductionRatio)) * K_WheelCircufrence );
    V_Cnt_WheelDeltaDistancePrev[index]  = V_Cnt_WheelDeltaDistanceCurr[index];
    }

  V_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / K_ReductionRatio) / 60) * K_WheelCircufrence;
  V_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / K_ReductionRatio) / 60) * K_WheelCircufrence;

  V_TurretPosition = L_encoderTurretAngle * (-K_k_TurretEncoderScaler); // Negative to rotate the output.  Positive is clockwise when viewed from top.
  }