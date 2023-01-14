/*
  LookUp.hpp

   Created on: Feb 14, 2018
       Author: biggs
 */
#ifndef SRC_ROBORIO2018_LOOKUP_HPP_
#define SRC_ROBORIO2018_LOOKUP_HPP_
extern double ScaleJoystickAxis(double L_JoystickAxis);

extern double RampTo(double  L_Final,
                     double  L_Current,
                     double  L_Slope);

extern void DesiredRollerSpeed(double  L_Distance,
                               double  L_Angle,
                               double *L_UpperCmnd,
                               double *L_LowerCmnd);

extern double DesiredRotateSpeed(double L_Error);

extern double DesiredAutoRotateSpeed(double L_Error);

extern double DesiredLowerBeamSpeed(double L_TargetDistance);

extern double DesiredUpperBeamSpeed(double L_TargetDistance);

extern double DtrmnAutoLauncherSpeed(double L_TargetDistance);

extern double DtrmnTimeToDriveToCaptureBall(double L_EstTargetDistance);

extern void DesiredAutonLocation(double  L_t_AutonTime,
                                 double *L_L_X_Location,
                                 double *L_L_Y_Location);

bool DesiredAutonLocation2(double  L_t_AutonTime,
                           int     L_int_AutonSelection,
                           double *L_L_X_Location,
                           double *L_L_Y_Location,
                           double *L_Deg_Angle);
#endif /* SRC_ROBORIO2018_LOOKUP_HPP_ */
