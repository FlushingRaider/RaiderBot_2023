/*
  Encoders.hpp

  Created on: Feb 25, 2020

  Author: 5561

  Updates:
  2022-02-15: Cleaned up file
*/

 extern double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz];
 extern double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; 
 extern double VaENC_Deg_WheelAngleRev[E_RobotCornerSz];
 extern double VaENC_In_WheelDeltaDistance[E_RobotCornerSz];
 extern double VeENC_Deg_TurretPosition;
 
 void Encoders_Drive_CompBot(double                       LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                             double                       LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                             double                       LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                             double                       LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                             rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                             rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                             rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                             rev::SparkMaxRelativeEncoder m_encoderRearRightDrive);

void Encoders_Drive_PracticeBot(double                       LeENC_Cnt_EncoderWheelAngleFrontLeftRawPracticeBot,
                    double                       LeENC_Cnt_EncoderWheelAngleFrontRightRawPracticeBot,
                    double                       LeENC_Cnt_EncoderWheelAngleRearLeftRawPracticeBot,
                    double                       LeENC_Cnt_EncoderWheelAngleRearRightRawPracticeBot,
                    rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                    rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                    rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                    rev::SparkMaxRelativeEncoder m_encoderRearRightDrive);
                      
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

                      
                      
void Encoders_MAN_INT( rev::SparkMaxRelativeEncoder m_IntakeRollersEncoder,
                       rev::SparkMaxRelativeEncoder m_ArmPivotEncoder,
                       rev::SparkMaxRelativeEncoder m_GripperEncoder,
                       rev::SparkMaxRelativeEncoder m_WristEncoder,
                       double LeENC_Deg_LinearSlide,
                       double LeENC_Deg_EncoderTurretRotate);