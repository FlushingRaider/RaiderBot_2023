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
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <photonlib/PhotonPoseEstimator.h>
#include <VisionV2.hpp>

#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpinet/PortForwarder.h>
#include "Odometry.hpp"

// all our favorite variables
// double         V_VisionTopCamNumberTemp = 1; // temporary fix for cams flipping, may not be needed
int VnVIS_int_VisionCameraIndex[E_CamSz]; //
T_CameraNumber VnVIS_e_VisionCamNumber[E_CamLocSz];

// bool VeVIS_b_VisionTargetAquired[E_CamLocSz]; 
bool VeVIS_b_VisionTargetAquired;
double VeVIS_Deg_VisionYaw[E_CamLocSz];
double VeVIS_m_VisionTargetDistance[E_CamLocSz];

bool VeVIS_b_VisionDriverRequestedModeCmnd;        // Requested driver mode override
bool VeVIS_b_VisionDriverRequestedModeCmndLatched; // Latched state of the driver requested mode
bool VeVIS_b_VisionDriverRequestedModeCmndPrev;    // Requested driver mode override previous

bool VeVIS_b_VisionDriverModeCmndFinal; // Final command to toggle the camera driver mode

#ifdef NewVision
bool VeVIS_b_TagHasTarget;
double V_Tagx;
double V_Tagy;
double V_Tagz;
double V_TagRoll;
double V_TagPitch;
double V_TagYaw;
int V_TagID;

photonlib::PhotonCamera Cam1 = photonlib::PhotonCamera("Cam1"); // the string is the name of the cam from photon, the name in photon must match this one to assign properly
fs::path aprilTagsjsonPath = frc::filesystem::GetDeployDirectory();
std::string aprilTagsjson = aprilTagsjsonPath / "2023_chargedUp.json";

frc::AprilTagFieldLayout aprilTags{aprilTagsjson};

//  photonlib::PhotonPoseEstimator *estimator;;
photonlib::PhotonPoseEstimator estimator(aprilTags, photonlib::LOWEST_AMBIGUITY, std::move(Cam1), {});

#endif

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
void VisionInit(frc::DriverStation::Alliance LeLC_e_AllianceColor)
{
  // /* Check the driver station for what the top camera number should be: */
  // V_VisionTopCamNumberTemp = frc::SmartDashboard::GetNumber("Top Camera Number", V_VisionTopCamNumberTemp);

  // if (fabs(V_VisionTopCamNumberTemp) < 1.5)
  //   {
  //   VnVIS_e_VisionCamNumber[E_CamTop] = E_Cam1;
  //   VnVIS_e_VisionCamNumber[E_CamBottom] = E_Cam2;
  //   }
  // else
  //   {
  //   VnVIS_e_VisionCamNumber[E_CamTop] = E_Cam2;
  //   VnVIS_e_VisionCamNumber[E_CamBottom] = E_Cam1;
  //   }

  /******************************************************************************
   *  VERY IMPORTANT COMMENTS!!!
   *
   *  photonPoseEstimator has no c++ example code on the photon docs and I had to find this
   *   in the unit tests of photonvision
   ******************************************************************************

     frc::AprilTagFieldLayout aprilTags(aprilTagsJson);

      photonlib::PhotonPoseEstimator estimator(
        aprilTags, photonlib::LOWEST_AMBIGUITY, std::move(Cam1), {});
    auto estimatedPose = estimator.Update();
    frc::Pose3d pose = estimatedPose.value().estimatedPose;

  static std::vector<frc::AprilTag> tags = {
      {0, frc::Pose3d(units::meter_t(3), units::meter_t(3), units::meter_t(3),
                      frc::Rotation3d())},
      {1, frc::Pose3d(units::meter_t(5), units::meter_t(5), units::meter_t(5),
                      frc::Rotation3d())}};

  static frc::AprilTagFieldLayout aprilTags{aprilTagsjson.string()};

   photonlib::PhotonPoseEstimator estimator(
        aprilTags, photonlib::LOWEST_AMBIGUITY, std::move(Cam1), {});

  ****************************************/

#ifdef OldVision
  VnVIS_e_VisionCamNumber[E_CamTop] = E_Cam1;
  VnVIS_e_VisionCamNumber[E_CamBottom] = E_Cam2;

  // gets flag from the driver station to choose between alliance colors

  if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
  {
    VnVIS_int_VisionCameraIndex[VnVIS_e_VisionCamNumber[E_CamBottom]] = 0; // 1 is the index for a red ball
    VnVIS_int_VisionCameraIndex[VnVIS_e_VisionCamNumber[E_CamTop]] = 1;    // 1 is the top camera targeting index
  }
  else // if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kBlue) -> must be either red or blue
  {
    VnVIS_int_VisionCameraIndex[VnVIS_e_VisionCamNumber[E_CamBottom]] = 1; // 2 is the index for a blue ball
    VnVIS_int_VisionCameraIndex[VnVIS_e_VisionCamNumber[E_CamTop]] = 1;    // 1 is the top camera targeting index
  }
#endif

#ifdef NewVision

#endif
}

/******************************************************************************
 * Function:     VisionRun
 *
 * Description:  Contains the necessary code relative to processing the
 *               vision output periodically.
 ******************************************************************************/
#ifdef OldVision
void VisionRun(photonlib::PhotonPipelineResult LsVIS_Str_TopResult,
               photonlib::PhotonPipelineResult LsVIS_Str_BottomResult,
               bool LbVIS_b_AutoTargetRequest,
               bool LbVIS_b_DriverDriveModeReq,
               bool *LbVIS_b_VisionDriverModeCmndFinal)
{

  T_CameraLocation LeVIS_Int_Index = E_CamTop;
  units::meter_t LeVIS_m_Range = 0_m;
  photonlib::PhotonTrackedTarget LsVIS_Str_Target;
  photonlib::PhotonPipelineResult pc_LsVIS_Str_Result[E_CamSz];
  // bool L_VisionDriverModeCmndFinalTemp = false;

  pc_LsVIS_Str_Result[E_Cam1] = LsVIS_Str_TopResult;
  pc_LsVIS_Str_Result[E_Cam2] = LsVIS_Str_BottomResult;

  /* Check to see what the driver wants for the driver mode: */
  if ((LbVIS_b_DriverDriveModeReq != VeVIS_b_VisionDriverRequestedModeCmndPrev) && (LbVIS_b_DriverDriveModeReq == true))
  {
    /* Ok, we seem to have experienced a button press.  Let's flip the latched state*/
    if (VeVIS_b_VisionDriverRequestedModeCmndLatched == true)
    {
      VeVIS_b_VisionDriverRequestedModeCmndLatched = false; // When false, the driver is requesting to revert to the targeting overlay
    }
    else
    {
      VeVIS_b_VisionDriverRequestedModeCmndLatched = true; // When true, the driver is wanting a clear picture.
    }
  }

  /* Save the previous version to help determine when there is a transition in the driver button press. */
  VeVIS_b_VisionDriverRequestedModeCmndPrev = LbVIS_b_DriverDriveModeReq;

  /* Let's do the calculations here: */
  if ((VeVIS_b_VisionDriverRequestedModeCmndLatched == false))
  {
    /* Ok, we want vision to be active and sending data: */
    for (LeVIS_Int_Index = E_CamTop;
         LeVIS_Int_Index < E_CamLocSz;
         LeVIS_Int_Index = T_CameraLocation(int(LeVIS_Int_Index) + 1))
    {
      VeVIS_b_VisionTargetAquired[LeVIS_Int_Index] = pc_LsVIS_Str_Result[VnVIS_e_VisionCamNumber[LeVIS_Int_Index]].HasTargets(); // returns true if the camera has a target

      if (VeVIS_b_VisionTargetAquired[LeVIS_Int_Index] == true)
      {
        LsVIS_Str_Target = pc_LsVIS_Str_Result[VnVIS_e_VisionCamNumber[LeVIS_Int_Index]].GetBestTarget(); // gets the best target

        // VeVIS_Deg_VisionYaw[LeVIS_Int_Index] = LsVIS_Str_Target.GetYaw(); // Yaw of the best target

        VeVIS_Deg_VisionYaw[LeVIS_Int_Index] = Filter_FirstOrderLag(LsVIS_Str_Target.GetYaw(), VeVIS_Deg_VisionYaw[LeVIS_Int_Index], K_VisionYawLagFilter[LeVIS_Int_Index]);

        LeVIS_m_Range = photonlib::PhotonUtils::CalculateDistanceToTarget(
            K_VisionHeight[LeVIS_Int_Index], K_VisionTargetHeight[LeVIS_Int_Index], K_VisionCameraPitch[LeVIS_Int_Index],
            units::degree_t{pc_LsVIS_Str_Result[VnVIS_e_VisionCamNumber[LeVIS_Int_Index]].GetBestTarget().GetPitch()}); // first 3 variables are constants from Const.hpp

        if (LeVIS_m_Range < 0_m)
        {
          LeVIS_m_Range = 0_m;
        }
        // VeVIS_m_VisionTargetDistance[LeVIS_Int_Index] = LeVIS_m_Range.value();

        VeVIS_m_VisionTargetDistance[LeVIS_Int_Index] = Filter_FirstOrderLag(LeVIS_m_Range.value(), VeVIS_m_VisionTargetDistance[LeVIS_Int_Index], K_VisionTargetDistLagFilter[LeVIS_Int_Index]);
      }
    }
  }
  else
  {
    for (LeVIS_Int_Index = E_CamTop;
         LeVIS_Int_Index < E_CamLocSz;
         LeVIS_Int_Index = T_CameraLocation(int(LeVIS_Int_Index) + 1))
    {
      /* We don't want to be active, default everything to indicate no data: */
      VeVIS_b_VisionTargetAquired[LeVIS_Int_Index] = false;
      VeVIS_Deg_VisionYaw[LeVIS_Int_Index] = 0;
      VeVIS_m_VisionTargetDistance[LeVIS_Int_Index] = 0;
    }
  }

  /* Send the command out to photon vision: */
  *LbVIS_b_VisionDriverModeCmndFinal = VeVIS_b_VisionDriverRequestedModeCmndLatched;
}
#endif

#ifdef NewVision
void VisionRun()
{
  // wpi::PortForwarder::GetInstance().Add(5800, "10.55.61.11", 5800);
  photonlib::PhotonPipelineResult CamResult = estimator.GetCamera().GetLatestResult(); // GetCamera() pulls the camresult from the estimator value

  VeVIS_b_TagHasTarget = CamResult.HasTargets();

  if (VeVIS_b_TagHasTarget)
  {

    auto estimatedPose = estimator.Update();
    frc::Pose3d pose = estimatedPose.value().estimatedPose; // "pose" object which holds xyz and roll,pitch,yaw values
    V_TagID = CamResult.GetBestTarget().GetFiducialId();

    V_Tagx = pose.X().value();
    V_Tagy = pose.Y().value();
    V_Tagz = pose.Z().value();
    V_TagRoll = pose.Rotation().X().value();
    V_TagPitch = pose.Rotation().Y().value();
    V_TagYaw = pose.Rotation().Z().value();

    OdometryInitToArgs(V_Tagx, V_Tagy);
  }
  else
  {

    // zero values when no target
    V_TagID = 0;

    V_Tagx = 0.0;
    V_Tagy = 0.0;
    V_Tagz = 0.0;
    V_TagRoll = 0.0;
    V_TagPitch = 0.0;
    V_TagYaw = 0.0;
  }
}

#endif