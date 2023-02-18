/*
  Odometry.hpp

  Created on: Feb 17, 2021
  Author: Biggs

  Contains the code related to the odemetry tracking of the robot.
 */

extern double VeODO_In_RobotDisplacementX;
extern double VeODO_In_RobotDisplacementY;

void OdometryInitToArgs(double VeODO_In_ArgX, double VeODO_In_ArgY);

void OdometryInit(void);

void DtrmnSwerveBotLocation(double L_Rad_Gyro,
                            double *L_Rad_WheelAngleFwd,
                            double *LeODO_In_DeltaWheelDistance,
                            bool LeODO_b_ResetButton);