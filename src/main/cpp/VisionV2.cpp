/*
  VisionV2.cpp

  Created on: Feb 2022
  Author: Carson

  Changes:
  
 */

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include "Const.hpp"
#include "Filter.hpp"










// all our favorite variables
double         V_VisionTopCamNumberTemp = 1;
int            V_VisionCameraIndex[E_CamSz];
T_CameraNumber V_VisionCamNumber[E_CamLocSz];

bool           V_VisionTargetAquired[E_CamLocSz];
double         V_VisionYaw[E_CamLocSz];
double         V_VisionTargetDistanceMeters[E_CamLocSz];

double         V_VisionDriverModeDelayTime; // Time to delay before allowing calculations.  This is needed to allow the driver mode to be communicated with the PI and then to allow the data to come back once in the correct mode.

bool           V_VisionDriverRequestedModeCmnd; // Requested driver mode override
bool           V_VisionDriverRequestedModeCmndLatched; // Latched state of the driver requested mode
bool           V_VisionDriverRequestedModeCmndPrev; // Requested driver mode override previous

bool           V_VisionDriverModeCmndFinal; // Final command to toggle the camera driver mode

/******************************************************************************
 * Function:     VisionRobotInit
 *
 * Description:  Initializes the camera selector at robot init.
 ******************************************************************************/
void VisionRobotInit()
  {
  /* Place an input on the dash.  A value of 1 indicates top camera is cam 1, 
     otherwise it is camera 2 */
  // frc::SmartDashboard::PutNumber("Top Camera Number", V_VisionTopCamNumberTemp);
  }


/******************************************************************************
 * Function:     VisionInit
 *
 * Description:  Initialize vision and related variables.
 ******************************************************************************/
void VisionInit(frc::DriverStation::Alliance L_AllianceColor)
  {
  // /* Check the driver station for what the top camera number should be: */
  // V_VisionTopCamNumberTemp = frc::SmartDashboard::GetNumber("Top Camera Number", V_VisionTopCamNumberTemp);

  // if (fabs(V_VisionTopCamNumberTemp) < 1.5)
  //   {
  //   V_VisionCamNumber[E_CamTop] = E_Cam1;
  //   V_VisionCamNumber[E_CamBottom] = E_Cam2;
  //   }
  // else
  //   {
  //   V_VisionCamNumber[E_CamTop] = E_Cam2;
  //   V_VisionCamNumber[E_CamBottom] = E_Cam1;
  //   }

  V_VisionCamNumber[E_CamTop] = E_Cam1;
  V_VisionCamNumber[E_CamBottom] = E_Cam2;

  // gets flag from the driver station to choose between alliance colors
  if (L_AllianceColor == frc::DriverStation::Alliance::kRed)
    {
    V_VisionCameraIndex[V_VisionCamNumber[E_CamBottom]] = 0; // 1 is the index for a red ball
    V_VisionCameraIndex[V_VisionCamNumber[E_CamTop]] = 1; // 1 is the top camera targeting index
    }
  else // if (L_AllianceColor == frc::DriverStation::Alliance::kBlue) -> must be either red or blue
    {
    V_VisionCameraIndex[V_VisionCamNumber[E_CamBottom]] = 1; // 2 is the index for a blue ball
    V_VisionCameraIndex[V_VisionCamNumber[E_CamTop]] = 1; // 1 is the top camera targeting index
    }
  }


/******************************************************************************
 * Function:     VisionRun
 *
 * Description:  Contains the necessary code relative to processing the 
 *               vision output.
 ******************************************************************************/
void VisionRun(photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult,
               bool                            L_AutoTargetRequest,
               bool                            L_DriverDriveModeReq,
               bool                           *L_VisionDriverModeCmndFinal)
  {
  T_CameraLocation L_Index = E_CamTop;
  units::meter_t L_Range = 0_m;
  photonlib::PhotonTrackedTarget L_Target;
  photonlib::PhotonPipelineResult pc_L_Result[E_CamSz];
  bool L_VisionDriverModeCmndFinalTemp = false;

  pc_L_Result[E_Cam1] = pc_L_TopResult;
  pc_L_Result[E_Cam2] = pc_L_BottomResult;
  
  /* Check to see what the driver wants for the driver mode: */
  if ((L_DriverDriveModeReq != V_VisionDriverRequestedModeCmndPrev) && (L_DriverDriveModeReq == true))
    {
    /* Ok, we seem to have experienced a button press.  Let's flip the latched state*/
    if (V_VisionDriverRequestedModeCmndLatched == true)
      {
      V_VisionDriverRequestedModeCmndLatched = false; // When false, the driver is requesting to revert to the targeting overlay
      }
    else
      {
      V_VisionDriverRequestedModeCmndLatched = true;  // When true, the driver is wanting a clear picture.
      }
    }

  /* Save the previous version to help determine when there is a transition in the driver button press. */
  V_VisionDriverRequestedModeCmndPrev = L_DriverDriveModeReq;

  /* Let's do the calculations here: */
  if ((V_VisionDriverRequestedModeCmndLatched == false))
    {
    /* Ok, we want vision to be active and sending data: */
    for (L_Index = E_CamTop;
         L_Index < E_CamLocSz;
         L_Index = T_CameraLocation(int(L_Index) + 1))
      {
      V_VisionTargetAquired[L_Index] = pc_L_Result[V_VisionCamNumber[L_Index]].HasTargets(); //returns true if the camera has a target  
    
      if (V_VisionTargetAquired[L_Index] == true)
        {
        L_Target = pc_L_Result[V_VisionCamNumber[L_Index]].GetBestTarget(); //gets the best target  
    
        // V_VisionYaw[L_Index] = L_Target.GetYaw(); // Yaw of the best target

        V_VisionYaw[L_Index] = Filter_FirstOrderLag(L_Target.GetYaw(), V_VisionYaw[L_Index], K_VisionYawLagFilter[L_Index]);
      
        L_Range = photonlib::PhotonUtils::CalculateDistanceToTarget(
                     K_VisionHeight[L_Index], K_VisionTargetHeight[L_Index], K_VisionCameraPitch[L_Index],
                     units::degree_t{pc_L_Result[V_VisionCamNumber[L_Index]].GetBestTarget().GetPitch()}); // first 3 variables are constants from Const.hpp  
        
        if (L_Range < 0_m)
          {
          L_Range = 0_m;
          }
        // V_VisionTargetDistanceMeters[L_Index] = L_Range.value();

        V_VisionTargetDistanceMeters[L_Index] = Filter_FirstOrderLag(L_Range.value(), V_VisionTargetDistanceMeters[L_Index], K_VisionTargetDistLagFilter[L_Index]);
        }
      }
    }
  else
    {
    for (L_Index = E_CamTop;
         L_Index < E_CamLocSz;
         L_Index = T_CameraLocation(int(L_Index) + 1))
      {
      /* We don't want to be active, default everything to indicate no data: */
      V_VisionTargetAquired[L_Index] = false;
      V_VisionYaw[L_Index] = 0;
      V_VisionTargetDistanceMeters[L_Index] = 0;
      }
    }

  /* Send the command out to photon vision: */
  *L_VisionDriverModeCmndFinal = V_VisionDriverRequestedModeCmndLatched;
  }
