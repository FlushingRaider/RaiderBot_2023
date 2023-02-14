/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561
 */

#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"

#include "Const.hpp"

double VaENC_Deg_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the angle Encoder and processed to only be from 0 - 180
double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double VaENC_Deg_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double VaENC_InS_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double VaENC_In_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double VaENC_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Current distance wheel moved, loop to loop, in Counts
double VaENC_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts
double VeENC_RPM_ShooterSpeedCurr; //TODO: not currently used, need to remove later

double VeENC_In_LiftPostitionYD; // Position of the Y Direction lift
double VeENC_In_LiftPostitionXD; // Position of the X Direction lift

double VeENC_Deg_TurretPosition; // Position of the turrent rotation
double VeENC_Deg_ArmPivot; // Posistion of the Arm Angle
double VeENC_RPM_IntakeRollers; // Speed of the intake rollers
double VeENC_Deg_Gripper; // Position of the gripper
double VeENC_Deg_Wrist; // Actual position of the wrist.

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
    T_RobotCorner LeENC_e_Index;

    for (LeENC_e_Index = E_FrontLeft;
         LeENC_e_Index < E_RobotCornerSz;
         LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
      {
        VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = 0;
        VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = 0;
        VaENC_InS_WheelVelocity[LeENC_e_Index] = 0;
        VaENC_In_WheelDeltaDistance[LeENC_e_Index] = 0;
        VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index] = 0;
        VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index] = 0;
      }

    m_encoderFrontRightSteer.SetPosition(0);
    m_encoderFrontLeftSteer.SetPosition(0);
    m_encoderRearRightSteer.SetPosition(0);
    m_encoderRearLeftSteer.SetPosition(0);

    m_encoderFrontRightDrive.SetPosition(0);
    m_encoderFrontLeftDrive.SetPosition(0);
    m_encoderRearRightDrive.SetPosition(0);
    m_encoderRearLeftDrive.SetPosition(0);

    VeENC_RPM_ShooterSpeedCurr = 0;
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
double EncoderAngleLimit(double LeENC_Cnt_EncoderRawValue,
                         double LeENC_k_EncoderRawToAngleCal,
                         double LeENC_Deg_EncoderOffset,
                         double LeENC_Deg_EncoderLimit)
  {
  double LeENC_Deg_EncoderConverted = 0;

  LeENC_Deg_EncoderConverted = LeENC_Cnt_EncoderRawValue * LeENC_k_EncoderRawToAngleCal - LeENC_Deg_EncoderLimit + LeENC_Deg_EncoderOffset;

  if (LeENC_Deg_EncoderConverted > LeENC_Deg_EncoderLimit)
    {
      LeENC_Deg_EncoderConverted -= (2 * LeENC_Deg_EncoderLimit);
    }
  else if (LeENC_Deg_EncoderConverted < -LeENC_Deg_EncoderLimit)
    {
      LeENC_Deg_EncoderConverted += (2 * LeENC_Deg_EncoderLimit);
    }
    
  return (LeENC_Deg_EncoderConverted);
  }


/******************************************************************************
 * Function:     Encoders_Drive_CompBot
 *
 * Description:  Run all of the encoder decoding logic.
 ******************************************************************************/
void Encoders_Drive_CompBot(double                       LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                            double                       LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                            double                       LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                            double                       LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                            rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                            rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                            rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                            rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                            rev::SparkMaxRelativeEncoder m_IntakeArmEncoder,
                            rev::SparkMaxRelativeEncoder m_IntakeRollersEncoder,
                            rev::SparkMaxRelativeEncoder m_GripperEncoder,
                            rev::SparkMaxRelativeEncoder m_WristEncoder,
                            double                       LeENC_Cnt_EncoderTurretAngle)
  {
  T_RobotCorner LeENC_e_Index;

  // VeENC_In_LiftPostitionYD = m_encoderLiftYD.GetPosition();
  // VeENC_In_LiftPostitionXD = m_encoderLiftXD.GetPosition();

  VaENC_Deg_WheelAngleConverted[E_FrontLeft]  = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontLeftRaw  * KeENC_k_EncoderToAngle), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontLeft];
  VaENC_Deg_WheelAngleConverted[E_FrontRight] = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontRightRaw * KeENC_k_EncoderToAngle), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontRight];
  VaENC_Deg_WheelAngleConverted[E_RearLeft]   = std::fmod((LeENC_Cnt_EncoderWheelAngleRearLeftRaw   * KeENC_k_EncoderToAngle), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearLeft];
  VaENC_Deg_WheelAngleConverted[E_RearRight]  = std::fmod((LeENC_Cnt_EncoderWheelAngleRearRightRaw  * KeENC_k_EncoderToAngle), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearRight];

  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();

  for (LeENC_e_Index = E_FrontLeft;
       LeENC_e_Index < E_RobotCornerSz;
       LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
    {
    VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleConverted[LeENC_e_Index];

    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] > 180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] -= 360;
      }
    else if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] < -180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] >= 0)
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] - 180;
      }
    else
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] * (C_PI/180);

    VaENC_In_WheelDeltaDistance[LeENC_e_Index]  = ((((VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index]  - VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index])/  KeENC_k_ReductionRatio)) * KeENC_In_WheelCircumfrence );
    VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index]  = VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index];
    }

  VaENC_InS_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;

 VeENC_RPM_ShooterSpeedCurr = m_encoderrightShooter.GetVelocity(); // We use the right shooter as the reference as this is rotating in the positive direction

  VeENC_Deg_TurretPosition = LeENC_Cnt_EncoderTurretAngle * (-KeENC_k_TurretEncoderScaler); // Negative to rotate the output.  Positive is clockwise when viewed from top.
  }


  /******************************************************************************
 * Function:     Encoders_MAN_INT
 *
 * Description:  Read the encoders from the Manipulator and Intake
 ******************************************************************************/
void Encoders_MAN_INT( rev::SparkMaxRelativeEncoder m_IntakeRollersEncoder,
                       rev::SparkMaxRelativeEncoder m_ArmPivotEncoder,
                       rev::SparkMaxRelativeEncoder m_GripperEncoder,
                       rev::SparkMaxRelativeEncoder m_WristEncoder,
                       double LeENC_Deg_LinearSlide,
                       double LeENC_Deg_EncoderTurretRotate)
  {
  T_RobotCorner LeENC_e_Index;

VeENC_Deg_ArmPivot = m_ArmPivotEncoder.GetPosition() * KeENC_k_Armpivot;

VeENC_RPM_IntakeRollers = m_IntakeRollersEncoder.GetVelocity() * KeENC_RPM_IntakeRollers;

VeENC_Deg_Gripper = m_GripperEncoder.GetPosition() * KeENC_Deg_Gripper;

VeENC_Deg_Wrist = m_WristEncoder.GetPosition() * KeENC_Deg_Wrist;

 VeENC_RPM_ShooterSpeedCurr = m_encoderrightShooter.GetVelocity(); // We use the right shooter as the reference as this is rotating in the positive direction

  VeENC_Deg_TurretPosition = LeENC_Deg_EncoderTurretRotate * (-KeENC_k_TurretEncoderScaler); // Negative to rotate the output.  Positive is clockwise when viewed from top.
  }



/******************************************************************************
 * Function:     Encoders_Drive_PracticeBot
 *
 * Description:  Run all of the encoder decoding logic, for practice bot.
 ******************************************************************************/
void Encoders_Drive_PracticeBot(double                       LeENC_Cnt_EncoderWheelAngleFrontLeftRawPracticeBot,
                                double                       LeENC_Cnt_EncoderWheelAngleFrontRightRawPracticeBot,
                                double                       LeENC_Cnt_EncoderWheelAngleRearLeftRawPracticeBot,
                                double                       LeENC_Cnt_EncoderWheelAngleRearRightRawPracticeBot,
                                rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                                rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                                rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                                rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                                double                       LeENC_Cnt_EncoderTurretAngle)
  {
  T_RobotCorner LeENC_e_Index;

  VaENC_Deg_WheelAngleConverted[E_FrontLeft]  = EncoderAngleLimit(-LeENC_Cnt_EncoderWheelAngleFrontLeftRawPracticeBot,  KeENC_k_VoltageToAngle, KeENC_k_WheelOffsetAnglePractieBot[E_FrontLeft],  180);
  VaENC_Deg_WheelAngleConverted[E_FrontRight] = EncoderAngleLimit(-LeENC_Cnt_EncoderWheelAngleFrontRightRawPracticeBot, KeENC_k_VoltageToAngle, KeENC_k_WheelOffsetAnglePractieBot[E_FrontRight], 180);
  VaENC_Deg_WheelAngleConverted[E_RearLeft]   = EncoderAngleLimit(-LeENC_Cnt_EncoderWheelAngleRearLeftRawPracticeBot,   KeENC_k_VoltageToAngle, KeENC_k_WheelOffsetAnglePractieBot[E_RearLeft],   180);
  VaENC_Deg_WheelAngleConverted[E_RearRight]  = EncoderAngleLimit(-LeENC_Cnt_EncoderWheelAngleRearRightRawPracticeBot,  KeENC_k_VoltageToAngle, KeENC_k_WheelOffsetAnglePractieBot[E_RearRight],  180);

  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();
  
   frc::SmartDashboard::PutNumber("DeltaWheel_FL",  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]);
   frc::SmartDashboard::PutNumber("DeltaWheel_FR",  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontRight]);
   frc::SmartDashboard::PutNumber("DeltaWheel_RR",  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearRight]);
   frc::SmartDashboard::PutNumber("DeltaWheel_RL",  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearLeft]);

   frc::SmartDashboard::PutNumber("WheelAng_FL",  VaENC_Deg_WheelAngleConverted[E_FrontLeft]);
   frc::SmartDashboard::PutNumber("WheelAng_FR",  VaENC_Deg_WheelAngleConverted[E_FrontRight]);
   frc::SmartDashboard::PutNumber("WheelAng_RR",  VaENC_Deg_WheelAngleConverted[E_RearRight]);
   frc::SmartDashboard::PutNumber("WheelAng_RL",  VaENC_Deg_WheelAngleConverted[E_RearLeft]);

  for (LeENC_e_Index = E_FrontLeft;
       LeENC_e_Index < E_RobotCornerSz;
       LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
    {
    VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleConverted[LeENC_e_Index];

    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] > 180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] -= 360;
      }
    else if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] < -180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] >= 0)
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] - 180;
      }
    else
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] * (C_PI/180);

    // VaENC_In_WheelDeltaDistance[LeENC_e_Index]  = ((VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index]  - VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index]));
    VaENC_In_WheelDeltaDistance[LeENC_e_Index]  = (((VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index]  - VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index]) /  KeENC_k_ReductionRatio) * KeENC_In_WheelCircumfrence );
    VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index]  = VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index];
    }

  
  VaENC_InS_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;

  VeENC_Deg_TurretPosition = LeENC_Cnt_EncoderTurretAngle * (-KeENC_k_TurretEncoderScaler); // Negative to rotate the output.  Positive is clockwise when viewed from top.
  }