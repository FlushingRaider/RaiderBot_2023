/*
 * Odometry.cpp
 * 
 * Team 5561 2021 Code
 *
 * This code is meant track the field position of the robot based on the gyro and encoders:
 * - Started on 02/22/2021.  Built, untested.
 *
 * */

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Enums.hpp"
#include "Const.hpp"

double VeODO_In_RobotDisplacementX = 0;
double VeODO_In_RobotDisplacementY = 0;
double VeODO_In_DeltaWheelDistance[E_RobotCornerSz];


/******************************************************************************
 * Function:     OdometryInit
 *
 * Description:  Initializes the necessary items for odometry.
 *
 ******************************************************************************/
void OdometryInit()
  {
  VeODO_In_RobotDisplacementX = 0;
  VeODO_In_RobotDisplacementY = 0;
  }


/******************************************************************************
 * Function:     DtrmnSwerveBotLocation
 *
 * Description:  Tracks the location of the robot as it traverses the field.
 *
 ******************************************************************************/
void DtrmnSwerveBotLocation(double  LeODO_Rad_Gyro,
                            double *LeODO_Rad_WheelAngleArb,
                            double *LeODO_In_DeltaWheelDistance,
                            bool    LeODO_b_ResetButton,
                            double  *LaODO_k_WheelDirection)
  {
    T_RobotCorner LnODO_Cnt_Index;
    double        LeODO_Rad_RelativeAngle[E_RobotCornerSz];
    double        LeODO_In_DeltaCornerDisplacementX[E_RobotCornerSz];
    double        LeODO_In_DeltaCornerDisplacementY[E_RobotCornerSz];
    double        LeODO_In_TotalDeltaX = 0;
    double        LeODO_In_TotalDeltaY = 0;

    if (LeODO_b_ResetButton == true)
    {
    VeODO_In_RobotDisplacementX = 0;
    VeODO_In_RobotDisplacementY = 0;
    }

  for (LnODO_Cnt_Index = E_FrontLeft;
       LnODO_Cnt_Index < E_RobotCornerSz;
       LnODO_Cnt_Index = T_RobotCorner(int(LnODO_Cnt_Index) + 1))
    {
      LeODO_Rad_RelativeAngle[LnODO_Cnt_Index] = LeODO_Rad_Gyro + LeODO_Rad_WheelAngleArb[LnODO_Cnt_Index];
      LeODO_In_DeltaCornerDisplacementX[LnODO_Cnt_Index] = sin(LeODO_Rad_RelativeAngle[LnODO_Cnt_Index]) * LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index] * (LaODO_k_WheelDirection[LnODO_Cnt_Index]);
      LeODO_In_DeltaCornerDisplacementY[LnODO_Cnt_Index] = cos(LeODO_Rad_RelativeAngle[LnODO_Cnt_Index]) * LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index] * (LaODO_k_WheelDirection[LnODO_Cnt_Index]);

      LeODO_In_TotalDeltaX += LeODO_In_DeltaCornerDisplacementX[LnODO_Cnt_Index];
      LeODO_In_TotalDeltaY -= LeODO_In_DeltaCornerDisplacementY[LnODO_Cnt_Index];
      VeODO_In_DeltaWheelDistance[LnODO_Cnt_Index] += LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index];
    }
  
  frc::SmartDashboard::PutNumber("DeltaDistance_FL", VeODO_In_DeltaWheelDistance[E_FrontLeft]);
  frc::SmartDashboard::PutNumber("DeltaDistance_FR", VeODO_In_DeltaWheelDistance[E_FrontRight]);
  frc::SmartDashboard::PutNumber("DeltaDistance_RL", VeODO_In_DeltaWheelDistance[E_RearLeft]);
  frc::SmartDashboard::PutNumber("DeltaDistance_RR", VeODO_In_DeltaWheelDistance[E_RearRight]);

    frc::SmartDashboard::PutNumber("Dir_FL", LaODO_k_WheelDirection[E_FrontLeft]);
  frc::SmartDashboard::PutNumber("Dir_FR", LaODO_k_WheelDirection[E_FrontRight]);
  frc::SmartDashboard::PutNumber("Dir_RL", LaODO_k_WheelDirection[E_RearLeft]);
  frc::SmartDashboard::PutNumber("Dir_RR", LaODO_k_WheelDirection[E_RearRight]);

  LeODO_In_TotalDeltaX = LeODO_In_TotalDeltaX / 4;
  LeODO_In_TotalDeltaY = LeODO_In_TotalDeltaY / 4;

  VeODO_In_RobotDisplacementX += LeODO_In_TotalDeltaX;
  VeODO_In_RobotDisplacementY += LeODO_In_TotalDeltaY;
  }


