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

double V_l_RobotDisplacementX = 0;
double V_l_RobotDisplacementY = 0;


/******************************************************************************
 * Function:     OdometryInit
 *
 * Description:  Initializes the necessary items for odometry.
 *
 ******************************************************************************/
void OdometryInit()
  {
  V_l_RobotDisplacementX = 0;
  V_l_RobotDisplacementY = 0;
  }


/******************************************************************************
 * Function:     DtrmnSwerveBotLocation
 *
 * Description:  Tracks the location of the robot as it traverses the field.
 *
 ******************************************************************************/
void DtrmnSwerveBotLocation(double  L_Deg_Gyro,
                            double *L_Deg_WheelAngleArb,
                            double *L_M_DeltaWheelDistance)
  {
    T_RobotCorner L_e_Index;
    double        L_Deg_RelativeAngle[E_RobotCornerSz];
    double        L_M_DeltaCornerDisplacementX[E_RobotCornerSz];
    double        L_M_DeltaCornerDisplacementY[E_RobotCornerSz];
    double        L_M_TotalDeltaX = 0;
    double        L_M_TotalDeltaY = 0;

  for (L_e_Index = E_FrontLeft;
       L_e_Index < E_RobotCornerSz;
       L_e_Index = T_RobotCorner(int(L_e_Index) + 1))
    {
      L_Deg_RelativeAngle[L_e_Index] = L_Deg_Gyro + L_Deg_WheelAngleArb[L_e_Index];
      L_M_DeltaCornerDisplacementX[L_e_Index] = sin(L_Deg_RelativeAngle[L_e_Index]) * L_M_DeltaWheelDistance[L_e_Index];
      L_M_DeltaCornerDisplacementY[L_e_Index] = cos(L_Deg_RelativeAngle[L_e_Index]) * L_M_DeltaWheelDistance[L_e_Index];

      L_M_TotalDeltaX -= L_M_DeltaCornerDisplacementX[L_e_Index];
      L_M_TotalDeltaY -= L_M_DeltaCornerDisplacementY[L_e_Index];
    }
    
  L_M_TotalDeltaX = L_M_TotalDeltaX / 4;
  L_M_TotalDeltaY = L_M_TotalDeltaY / 4;

  V_l_RobotDisplacementX += L_M_TotalDeltaX;
  V_l_RobotDisplacementY += L_M_TotalDeltaY;
  }


