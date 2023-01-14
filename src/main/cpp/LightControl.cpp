/*
  LightControl.cpp

  Created on: Feb 15, 2022
  Author: Biggs

  This file contains functions related to control of lights on the robot.
  This can include but is not limited to:
   - LED vanity lights
   - Camera lights
 */

#include <frc/DriverStation.h>

#include "Const.hpp"

/* V_CameraLightOnTime: Indication of how long the light has been consecutivly on. */
double V_CameraLightOnTime = 0;

/* V_CameraLightStatus: Indication of the camera light status. */
T_CameraLightStatus V_CameraLightStatus = E_LightTurnedOff;

/* V_CameraLightCmndOn: Commanded camera light on/off state. */
bool V_CameraLightCmndOn = false;

/* V_VanityLightCmnd: PWM command to be sent to the blinkin controller. */
double  V_VanityLightCmnd = 0;

bool V_CameraLightLatch = false;

/******************************************************************************
 * Function:     CameraLightControl
 *
 * Description:  Contains the functionality for controlling the camera light.
 *               - Limits on time to prevent damaging light.
 *               - Informs targeting logic when camera feed should have had 
 *                 enough time with light on for accurate data.
 ******************************************************************************/
bool CameraLightControl(bool                 L_Driver_CameraLight,
                        T_ADAS_ActiveFeature L_ADAS_ActiveFeature,
                        bool                 L_ADAS_CameraUpperLightCmndOn)
  {
    bool L_CameraLightCmndOn = false;

    // if ((V_CameraLightLatch == false && L_Driver_CameraLight == true) ||
    //     ())
    //   {
    //   L_CameraLightCmndOn = true;
    //   V_CameraLightLatch = true;
    //   }

    if (((L_ADAS_ActiveFeature > E_ADAS_Disabled) &&    /* Swerve drive targeting has been requested or is in process */
         (L_ADAS_CameraUpperLightCmndOn == true)) ||
        (L_Driver_CameraLight == true))  /* Driver override is present */
      {
      L_CameraLightCmndOn = true;
      }

    if ((L_CameraLightCmndOn == true) &&
        (V_CameraLightOnTime < K_CameraLightMaxOnTime) &&
        (V_CameraLightStatus != E_LightForcedOffDueToOvertime))
      {
      V_CameraLightOnTime += C_ExeTime;

      if (V_CameraLightOnTime >= K_CameraLightDelay)
        {
        V_CameraLightStatus = E_LightOnTargetingReady;
        }
      else
        {
        V_CameraLightStatus = E_LightOnWaitingForTarget;
        }
      }
    else if ((L_CameraLightCmndOn == true) &&
             (V_CameraLightOnTime >= K_CameraLightMaxOnTime))
      {
        L_CameraLightCmndOn = false; // turn light off, give time to cool down
        V_CameraLightLatch = false;

        V_CameraLightStatus = E_LightForcedOffDueToOvertime;
      }
    else
      {
      V_CameraLightOnTime = 0;
      L_CameraLightCmndOn = false;
      V_CameraLightLatch = false;
      V_CameraLightStatus = E_LightTurnedOff;
      }
  
  /* Flip the command as the camera light is inverted */
  // if (L_CameraLightCmndOn == true)
  //   {
  //   L_CameraLightCmndOn = false;
  //   }
  // else
  //   {
  //   L_CameraLightCmndOn = true;
  //   }

  return (L_CameraLightCmndOn);
  }

/******************************************************************************
 * Function:     VanityLightControl
 *
 * Description:  Contains the functionality for controlling the vanity lights.
 *               - Changes colors based on alliance.
 *               - Will change color when in end game to help inform driver to
 *                 take action.
 ******************************************************************************/
double VanityLightControl(double                       L_MatchTimeRemaining,
                          frc::DriverStation::Alliance L_AllianceColor,
                          T_ADAS_ActiveFeature         L_ADAS_ActiveFeature,
                          bool                         L_ADAS_CameraLowerLightCmndOn,
                          bool                         L_Driver_CameraLight)
  {
    double L_LED_Command = 0;

    if (((L_ADAS_ActiveFeature > E_ADAS_Disabled) &&
         (L_ADAS_CameraLowerLightCmndOn == true)) ||
         (L_Driver_CameraLight == true))
      {
      L_LED_Command = C_BlinkinLED_SolidWhite;
      }
    else if ((L_MatchTimeRemaining <= C_End_game_time) &&
             (L_MatchTimeRemaining > 0))
      {
      L_LED_Command = C_BlinkinLED_RainbowWithGlitter;
      }
    else if (L_AllianceColor == frc::DriverStation::Alliance::kRed)
      {
      L_LED_Command = C_BlinkinLED_BreathRed;
      }
    else if (L_AllianceColor == frc::DriverStation::Alliance::kBlue)
      {
      L_LED_Command = C_BlinkinLED_BreathBlue;
      }
    else
      {
      L_LED_Command = C_BlinkinLED_LightChaseGray;
      }

    return(L_LED_Command);
  }

/******************************************************************************
 * Function:     LightControlMain
 *
 * Description:  Contains the functionality for controlling the camera 
 *               illumination lights and LED vanity lights.
 ******************************************************************************/
void LightControlMain(double                       L_MatchTimeRemaining,
                      frc::DriverStation::Alliance L_AllianceColor,
                      bool                         L_Driver_CameraLight,
                      T_ADAS_ActiveFeature         L_ADAS_ActiveFeature,
                      bool                         L_ADAS_CameraUpperLightCmndOn,
                      bool                         L_ADAS_CameraLowerLightCmndOn,
                      bool                        *L_CameraLightCmndOn,
                      double                      *L_VanityLightCmnd)
  {
  *L_CameraLightCmndOn = CameraLightControl(L_Driver_CameraLight,
                                            L_ADAS_ActiveFeature,
                                            L_ADAS_CameraUpperLightCmndOn);

  *L_VanityLightCmnd = VanityLightControl(L_MatchTimeRemaining,
                                          L_AllianceColor,
                                          L_ADAS_ActiveFeature,
                                          L_ADAS_CameraLowerLightCmndOn,
                                          L_Driver_CameraLight);
  }