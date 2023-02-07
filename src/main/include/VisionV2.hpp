/*
  VisionV2.hpp

   Created on: Feb 23, 2022
   Author: Carson

   Contains content from vision.
 */

#include <Enums.hpp>
#ifdef CarsonDebug

extern bool   VeVIS_b_VisionTargetAquired[E_CamLocSz];
extern double VeVIS_Deg_VisionYaw[E_CamLocSz];
extern double VeVIS_m_VisionTargetDistance[E_CamLocSz];
extern int    VnVIS_int_VisionCameraIndex[E_CamSz];
extern bool VeVIS_b_VisionDriverRequestedModeCmnd;

extern bool V_HasTarget;
extern double V_CamYaw;
extern double V_Tagx;
extern double V_Tagy;
extern double V_Tagz;
extern int V_TagID;
#endif


#ifdef OldVision

void VisionRobotInit();

void VisionInit(frc::DriverStation::Alliance LeLC_e_AllianceColor);

void VisionRun(photonlib::PhotonPipelineResult LsVIS_Str_TopResult,
               photonlib::PhotonPipelineResult LsVIS_Str_BottomResult,
               bool                            L_AutoTargetRequest,
               bool                            L_DriverDriveModeReq,
               bool                           *L_VisionDriverModeCmndFinal);
#endif

#ifdef CarsonDebug
void CarsonDebugRun();
#endif
