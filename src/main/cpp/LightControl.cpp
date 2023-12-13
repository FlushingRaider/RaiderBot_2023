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

/* VeLC_Cnt_CameraLightOnTime: Indication of how long the light has been consecutivly on. */
double VeLC_Cnt_CameraLightOnTime = 0;

/* VeLC_Sec_CameraLightStatus: Indication of the camera light status. */
T_CameraLightStatus VeLC_Sec_CameraLightStatus = E_LightTurnedOff;

/* VeLC_b_CameraLightCmndOn: Commanded camera light on/off state. */
bool VeLC_b_CameraLightCmndOn = false;

/* VeLC_Cmd_VanityLightCmnd: PWM command to be sent to the blinkin controller. */
double  VeLC_Cmd_VanityLightCmnd = 0;

bool VeLC_b_CameraLightLatch = false;

/******************************************************************************
 * Function:     CameraLightControl
 *
 * Description:  Contains the functionality for controlling the camera light.
 *               - Limits on time to prevent damaging light.
 *               - Informs targeting logic when camera feed should have had 
 *                 enough time with light on for accurate data.
 ******************************************************************************/
bool CameraLightControl(bool                 LeLC_b_Driver_CameraLight,
                        T_ADAS_ActiveFeature LeLC_e_ADASActiveFeature,
                        bool                 LeLC_b_ADASCameraUpperLightCmndOn)
  {
    bool LeLC_Cmd_CameraLightCmndOn = false;

    // if ((VeLC_b_CameraLightLatch == false && LeLC_b_Driver_CameraLight == true) ||
    //     ())
    //   {
    //   LeLC_Cmd_CameraLightCmndOn = true;
    //   VeLC_b_CameraLightLatch = true;
    //   }

    if (((LeLC_e_ADASActiveFeature > E_ADAS_Disabled) &&    /* Swerve drive targeting has been requested or is in process */
         (LeLC_b_ADASCameraUpperLightCmndOn == true)) ||
        (LeLC_b_Driver_CameraLight == true))  /* Driver override is present */
      {
      LeLC_Cmd_CameraLightCmndOn = true;
      }

    if ((LeLC_Cmd_CameraLightCmndOn == true) &&
        (VeLC_Cnt_CameraLightOnTime < KeLC_t_CameraLightMaxOnTime) &&
        (VeLC_Sec_CameraLightStatus != E_LightForcedOffDueToOvertime))
      {
      VeLC_Cnt_CameraLightOnTime += C_ExeTime;

      if (VeLC_Cnt_CameraLightOnTime >= KeLC_t_CameraLightDelay)
        {
        VeLC_Sec_CameraLightStatus = E_LightOnTargetingReady;
        }
      else
        {
        VeLC_Sec_CameraLightStatus = E_LightOnWaitingForTarget;
        }
      }
    else if ((LeLC_Cmd_CameraLightCmndOn == true) &&
             (VeLC_Cnt_CameraLightOnTime >= KeLC_t_CameraLightMaxOnTime))
      {
        LeLC_Cmd_CameraLightCmndOn = false; // turn light off, give time to cool down
        VeLC_b_CameraLightLatch = false;

        VeLC_Sec_CameraLightStatus = E_LightForcedOffDueToOvertime;
      }
    else
      {
      VeLC_Cnt_CameraLightOnTime = 0;
      LeLC_Cmd_CameraLightCmndOn = false;
      VeLC_b_CameraLightLatch = false;
      VeLC_Sec_CameraLightStatus = E_LightTurnedOff;
      }
  
  /* Flip the command as the camera light is inverted */
  // if (LeLC_Cmd_CameraLightCmndOn == true)
  //   {
  //   LeLC_Cmd_CameraLightCmndOn = false;
  //   }
  // else
  //   {
  //   LeLC_Cmd_CameraLightCmndOn = true;
  //   }

  return (LeLC_Cmd_CameraLightCmndOn);
  }

/******************************************************************************
 * Function:     VanityLightControl
 *
 * Description:  Contains the functionality for controlling the vanity lights.
 *               - Changes colors based on alliance.
 *               - Will change color when in end game to help inform driver to
 *                 take action.
 ******************************************************************************/
double VanityLightControl(double                       LeLC_Sec_MatchTimeRemaining,
                          frc::DriverStation::Alliance LeLC_e_AllianceColor,
                          T_ADAS_ActiveFeature         LeLC_e_ADASActiveFeature)
  {
    double LeLC_Cmd_LEDCommand = 0;

    if ((LeLC_Sec_MatchTimeRemaining <= C_End_game_time) &&
             (LeLC_Sec_MatchTimeRemaining > 0))
      {
      LeLC_Cmd_LEDCommand = CeLC_k_BlinkinLED_GreenAndRed;
      }
    else if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
      {
      LeLC_Cmd_LEDCommand = CeLC_k_BlinkinLED_BreathRed;
      }
    else if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kBlue)
      {
      LeLC_Cmd_LEDCommand = CeLC_k_BlinkinLED_BreathBlue;
      }
    else
      {
      LeLC_Cmd_LEDCommand = CeLC_k_BlinkinLED_GreenAndRed;
      }

    return(LeLC_Cmd_LEDCommand);
  }

/******************************************************************************
 * Function:     LightControlMain
 *
 * Description:  Contains the functionality for controlling the camera 
 *               illumination lights and LED vanity lights.
 ******************************************************************************/
void LightControlMain(double                       LeLC_Sec_MatchTimeRemaining,
                      frc::DriverStation::Alliance LeLC_e_AllianceColor,
                      bool                         LeLC_b_Driver_CameraLight,
                      T_ADAS_ActiveFeature         LeLC_e_ADASActiveFeature,
                      bool                         LeLC_b_ADASCameraUpperLightCmndOn,
                      bool                         LeLC_b_ADASCameraLowerLightCmndOn,
                      bool                        *LeLC_Cmd_CameraLightCmndOn,
                      double                      *LeLC_Cmd_VanityLightCmnd)
  {
  *LeLC_Cmd_CameraLightCmndOn = CameraLightControl(LeLC_b_Driver_CameraLight,
                                            LeLC_e_ADASActiveFeature,
                                            LeLC_b_ADASCameraUpperLightCmndOn);

  *LeLC_Cmd_VanityLightCmnd = VanityLightControl(LeLC_Sec_MatchTimeRemaining,
                                          LeLC_e_AllianceColor,
                                          LeLC_e_ADASActiveFeature);
  }