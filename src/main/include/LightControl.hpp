/*
  LightControl.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to control of lights on the robot.
  This can include but is not limited to:
   - LED vanity lights
   - Camera lights
 */

extern T_CameraLightStatus V_CameraLightStatus;
extern bool V_CameraLightCmndOn;
extern double  V_VanityLightCmnd;

void LightControlMain(double                       L_MatchTimeRemaining,
                      frc::DriverStation::Alliance L_AllianceColor,
                      bool                         L_Driver_CameraLight,
                      T_ADAS_ActiveFeature         L_ADAS_ActiveFeature,
                      bool                         L_ADAS_CameraUpperLightCmndOn,
                      bool                         L_ADAS_CameraLowerLightCmndOn,
                      bool                        *L_CameraLightCmndOn,
                      double                      *L_VanityLightCmnd);