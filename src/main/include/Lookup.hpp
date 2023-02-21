/*
  LookUp.hpp

   Created on: Feb 14, 2018
       Author: biggs
 */
#ifndef SRC_ROBORIO2018_LOOKUP_HPP_
#define SRC_ROBORIO2018_LOOKUP_HPP_

#include <string>

extern double ScaleJoystickAxis(double LeLU_Cmd_JoystickAxis);

extern double RampTo(double  L_Final,
                     double  L_Current,
                     double  L_Slope);

extern void DesiredRollerSpeed(double  LeLU_In_Distance,
                               double  LeLU_Deg_Angle,
                               double *LeLU_Cmd_UpperCmnd,
                               double *LeLU_Cmd_LowerCmnd);

extern double DesiredRotateSpeed(double LeLU_Cmd_Error);

extern double DesiredAutoRotateSpeed(double LeLU_Cmd_Error);

extern double DesiredLowerBeamSpeed(double LeLU_Cmd_TargetDistance);

extern double DesiredUpperBeamSpeed(double LeLU_Cmd_TargetDistance);

extern double DtrmnAutoLauncherSpeed(double LeLU_Cmd_TargetDistance);

extern double DtrmnTimeToDriveToCaptureBall(double LeLU_Cmd_EstTargetDistance);

extern void DesiredAutonLocation(double  LeLU_s_AutonTime,
                                 double *LeLU_Cmd_L_X_Location,
                                 double *LeLU_Cmd_L_Y_Location);

bool DesiredAutonLocation2(double  LeLU_s_AutonTime,
                           int     LeLU_Int_AutonSelection,
                           std::string V_ADAS_Auto_PathName,
                           double *LeLU_Cmd_L_X_Location,
                           double *LeLU_Cmd_L_Y_Location,
                           double *LeLU_Cmd_Deg_Angle);
#endif /* SRC_ROBORIO2018_LOOKUP_HPP_ */
