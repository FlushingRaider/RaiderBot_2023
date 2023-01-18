/*
  VisionV2.hpp

   Created on: Feb 23, 2022
   Author: Carson

   Contains content from vision.
 */

extern bool   VeVIS_b_VisionTargetAquired[E_CamLocSz];
extern double VeVIS_Deg_VisionYaw[E_CamLocSz];
extern double VeVIS_m_VisionTargetDistance[E_CamLocSz];
extern int    VnVIS_int_VisionCameraIndex[E_CamSz];
extern bool VeVIS_b_VisionDriverRequestedModeCmnd;

void VisionRobotInit();

void VisionInit(frc::DriverStation::Alliance L_AllianceColor);

void VisionRun(photonlib::PhotonPipelineResult LsVIS_Str_TopResult,
               photonlib::PhotonPipelineResult LsVIS_Str_BottomResult,
               bool                            L_AutoTargetRequest,
               bool                            L_DriverDriveModeReq,
               bool                           *L_VisionDriverModeCmndFinal);
