/*
  VisionV2.hpp

   Created on: Feb 23, 2022
   Author: Carson

   Contains content from vision.
 */

extern bool   V_VisionTargetAquired[E_CamLocSz];
extern double V_VisionYaw[E_CamLocSz];
extern double V_VisionTargetDistanceMeters[E_CamLocSz];
extern int    V_VisionCameraIndex[E_CamSz];
extern T_CameraNumber V_VisionCamNumber[E_CamLocSz];
extern double V_VisionTopCamNumberTemp;
extern bool V_VisionDriverModeCmndFinal;

void VisionRobotInit();

void VisionInit(frc::DriverStation::Alliance L_AllianceColor);

void VisionRun(photonlib::PhotonPipelineResult pc_L_TopResult,
               photonlib::PhotonPipelineResult pc_L_BottomResult,
               bool                            L_AutoTargetRequest,
               bool                            L_DriverDriveModeReq,
               bool                           *L_VisionDriverModeCmndFinal);
