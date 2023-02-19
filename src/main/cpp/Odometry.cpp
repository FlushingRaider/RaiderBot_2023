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

double VeODO_In_RobotDisplacementX = 0; // Displacement in the X direction, which is left/right
double VeODO_In_RobotDisplacementY = 0; // Displacement in the Y direction, which is forward/back

/******************************************************************************
 * Function:     OdometryInitToArgs
 *
 * Description:  Initializes the necessary items for odometry using a given pair of values
 *
 ******************************************************************************/
void OdometryInitToArgs(double VeODO_In_ArgX, double VeODO_In_ArgY)
{
  VeODO_In_RobotDisplacementX = VeODO_In_ArgX;
  VeODO_In_RobotDisplacementY = VeODO_In_ArgY;
}

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
void DtrmnSwerveBotLocation(double LeODO_Rad_Gyro,
                            double *LeODO_Rad_WheelAngleArb,
                            double *LeODO_In_DeltaWheelDistance,
                            bool LeODO_b_ResetButton)
{
  T_RobotCorner LnODO_Cnt_Index;
  double LeODO_Rad_RelativeAngle[E_RobotCornerSz];
  double LeODO_In_DeltaCornerDisplacementX[E_RobotCornerSz];
  double LeODO_In_DeltaCornerDisplacementY[E_RobotCornerSz];
  double LeODO_In_TotalDeltaX = 0;
  double LeODO_In_TotalDeltaY = 0;

  if (LeODO_b_ResetButton == true)
  {
    VeODO_In_RobotDisplacementX = 0;
    VeODO_In_RobotDisplacementY = 0;
  }

  for (LnODO_Cnt_Index = E_FrontLeft;
       LnODO_Cnt_Index < E_RobotCornerSz;
       LnODO_Cnt_Index = T_RobotCorner(int(LnODO_Cnt_Index) + 1))
  {
    LeODO_Rad_RelativeAngle[LnODO_Cnt_Index] = LeODO_Rad_Gyro - LeODO_Rad_WheelAngleArb[LnODO_Cnt_Index];
    LeODO_In_DeltaCornerDisplacementX[LnODO_Cnt_Index] = sin(LeODO_Rad_RelativeAngle[LnODO_Cnt_Index]) * LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index];
    LeODO_In_DeltaCornerDisplacementY[LnODO_Cnt_Index] = cos(LeODO_Rad_RelativeAngle[LnODO_Cnt_Index]) * LeODO_In_DeltaWheelDistance[LnODO_Cnt_Index];

    LeODO_In_TotalDeltaX += LeODO_In_DeltaCornerDisplacementX[LnODO_Cnt_Index];
    LeODO_In_TotalDeltaY -= LeODO_In_DeltaCornerDisplacementY[LnODO_Cnt_Index]; // Negative to have movement to the left as negative
  }

  LeODO_In_TotalDeltaX = LeODO_In_TotalDeltaX / 4;
  LeODO_In_TotalDeltaY = LeODO_In_TotalDeltaY / 4;

  VeODO_In_RobotDisplacementX += LeODO_In_TotalDeltaX;
  VeODO_In_RobotDisplacementY += LeODO_In_TotalDeltaY;
}
