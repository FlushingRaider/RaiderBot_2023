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

#include "Enums.hpp"

double VeODO_Cnt_RobotDisplacementX = 0;
double VeODO_Cnt_RobotDisplacementY = 0;


/******************************************************************************
 * Function:     OdometryInit
 *
 * Description:  Initializes the necessary items for odometry.
 *
 ******************************************************************************/
void OdometryInit()
  {
  VeODO_Cnt_RobotDisplacementX = 0;
  VeODO_Cnt_RobotDisplacementY = 0;
  }


/******************************************************************************
 * Function:     DtrmnSwerveBotLocation
 *
 * Description:  Tracks the location of the robot as it traverses the field.
 *
 ******************************************************************************/
void DtrmnSwerveBotLocation(double  LeODO_Deg_Gyro,
                            double *LeODO_Deg_WheelAngleArb,
                            double *LeODO_In_DeltaWheelDistance)
  {
    T_RobotCorner LnODO_Cnt_Index;
    double        LeODO_Deg_RelativeAngle[E_RobotCornerSz];
    double        LeODO_In_DeltaCornerDisplacementX[E_RobotCornerSz];
    double        LeODO_In_DeltaCornerDisplacementY[E_RobotCornerSz];
    double        LeODO_In_TotalDeltaX = 0;
    double        LeODO_In_TotalDeltaY = 0;

  for (LnODO_Cnt_Index = E_FrontLeft;
       LnODO_Cnt_Index < E_RobotCornerSz;
       LnODO_Cnt_Index = T_RobotCorner(int(LnODO_Cnt_Index) + 1))
    {
      LeODO_Deg_RelativeAngle[LnODO_Cnt_Index] = LeODO_Deg_Gyro + LeODO_Deg_WheelAngleArb[LnODO_Cnt_Index];
      LeODO_In_DeltaCornerDisplacementX[LnODO_Cnt_Index] = sin(LeODO_Deg_RelativeAngle[LnODO_Cnt_Index]) * LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index];
      LeODO_In_DeltaCornerDisplacementY[LnODO_Cnt_Index] = cos(LeODO_Deg_RelativeAngle[LnODO_Cnt_Index]) * LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index];

      LeODO_In_TotalDeltaX -= LeODO_In_DeltaCornerDisplacementX[LnODO_Cnt_Index];
      LeODO_In_TotalDeltaY -= LeODO_In_DeltaCornerDisplacementY[LnODO_Cnt_Index];
    }
    
  LeODO_In_TotalDeltaX = LeODO_In_TotalDeltaX / 4;
  LeODO_In_TotalDeltaY = LeODO_In_TotalDeltaY / 4;

  VeODO_Cnt_RobotDisplacementX += LeODO_In_TotalDeltaX;
  VeODO_Cnt_RobotDisplacementY += LeODO_In_TotalDeltaY;
  }


