/*
  Encoders.hpp

  Created on: Feb 25, 2020

  Author: 5561

  Updates:
  2022-02-15: Cleaned up file
*/

 extern double V_Deg_WheelAngleFwd[E_RobotCornerSz];
 extern double V_Rad_WheelAngleFwd[E_RobotCornerSz]; 
 extern double V_Deg_WheelAngleRev[E_RobotCornerSz];
 extern double V_WheelVelocity[E_RobotCornerSz];
 extern double V_M_WheelDeltaDistance[E_RobotCornerSz];
 extern double V_WheelAngleConverted[E_RobotCornerSz];
 extern double V_ShooterSpeedCurr;
 extern double V_LiftPostitionYD; // Position of the YD lift, in revolutions of the motor
 extern double V_LiftPostitionXD; // Position of the XD lift, in revolutions of the motor
 extern double V_TurretPosition;
 
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
                    double                       L_encoderTurretAngle);

void Read_Encoders2(double                       L_encoderWheelAngleFrontLeftRawPracticeBot,
                    double                       L_encoderWheelAngleFrontRightRawPracticeBot,
                    double                       L_encoderWheelAngleRearLeftRawPracticeBot,
                    double                       L_encoderWheelAngleRearRightRawPracticeBot,
                    rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                    rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                    rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                    rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                    double                       L_encoderTurretAngle);

void EncodersLiftInit(rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                      rev::SparkMaxRelativeEncoder m_encoderLiftXD);
                      
void EncodersInit(rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                  rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                  rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                  rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                  rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                  rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                  rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                  rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                  rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                  rev::SparkMaxRelativeEncoder m_encoderLiftXD,
                  rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                  rev::SparkMaxRelativeEncoder m_encoderleftShooter);


void EncodersInitCommon(rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive);

void EncodersInitComp(rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                      rev::SparkMaxRelativeEncoder m_encoderLiftXD,
                      rev::SparkMaxRelativeEncoder m_encoderrightShooter,
                      rev::SparkMaxRelativeEncoder m_encoderleftShooter);