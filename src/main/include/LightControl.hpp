/*
  LightControl.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to control of lights on the robot.
  This can include but is not limited to:
   - LED vanity lights
   - Camera lights
 */

extern T_CameraLightStatus VeLC_Sec_CameraLightStatus;
extern bool VeLC_b_CameraLightCmndOn;
extern double  VeLC_Cmd_VanityLightCmnd;

void LightControlMain(double                       LeLC_Sec_MatchTimeRemaining,
                      frc::DriverStation::Alliance LeLC_e_AllianceColor,
                      bool                         LeLC_b_Driver_CameraLight,
                      T_ADAS_ActiveFeature         LeLC_e_ADASActiveFeature,
                      bool                         LeLC_b_ADASCameraUpperLightCmndOn,
                      bool                         LeLC_b_ADASCameraLowerLightCmndOn,
                      bool                        *LeLC_Cmd_CameraLightCmndOn,
                      double                      *LeLC_Cmd_VanityLightCmnd);