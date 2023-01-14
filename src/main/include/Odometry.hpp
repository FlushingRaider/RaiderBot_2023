/*
  Odometry.hpp

  Created on: Feb 17, 2021
  Author: Biggs

  Contains the code related to the odemetry tracking of the robot.
 */

extern double V_l_RobotDisplacementX;
extern double V_l_RobotDisplacementY;

void OdometryInit(void);

void DtrmnSwerveBotLocation(double  L_Rad_Gyro,
                            double *L_Rad_WheelAngleFwd,
                            double *L_M_DeltaWheelDistance);