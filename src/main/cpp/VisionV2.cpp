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

#ifdef OldVision
// all our favorite variables
double V_VisionTopCamNumberTemp = 1; // temporary fix for cams flipping, may not be needed
int VnVIS_int_VisionCameraIndex[E_CamSz];
T_CameraNumber VnVIS_e_VisionCamNumber[E_CamLocSz];

bool VeVIS_b_VisionTargetAquired[E_CamLocSz];
double VeVIS_Deg_VisionYaw[E_CamLocSz];
double VeVIS_m_VisionTargetDistance[E_CamLocSz];

bool VeVIS_b_VisionDriverRequestedModeCmnd;        // Requested driver mode override
bool VeVIS_b_VisionDriverRequestedModeCmndLatched; // Latched state of the driver requested mode
bool VeVIS_b_VisionDriverRequestedModeCmndPrev;    // Requested driver mode override previous

bool VeVIS_b_VisionDriverModeCmndFinal; // Final command to toggle the camera driver mode
#endif
#ifdef NewVision
bool VeVIS_b_TagHasTarget;
double V_VIS_m_TagX;
double V_VIS_m_TagY;
double V_VIS_in_TagX;
double V_VIS_in_TagY;

double V_Tagz;
double V_TagRoll;
double V_TagPitch;
double V_TagYaw;
int V_TagID;
bool V_TagCentered;
bool V_CubeAlignRequested; 
bool V_ConeAlignRequested;

double NewStamp;
double OldStamp;
double OldX;
double OldY;

double XSpeed;
double YSpeed;


// vars for apriltag cam
photonlib::PhotonCamera Cam1 = photonlib::PhotonCamera("Cam1"); // the string is the name of the cam from photon, the name in photon must match this one to assign properly
fs::path aprilTagsjsonPath = frc::filesystem::GetDeployDirectory();
std::string aprilTagsjson = aprilTagsjsonPath / "2023_chargedUp.json"; //

frc::AprilTagFieldLayout aprilTags{aprilTagsjson};

photonlib::PhotonPoseEstimator estimator(aprilTags, photonlib::LOWEST_AMBIGUITY, std::move(Cam1), {});

photonlib::PhotonPipelineResult TagCamResult;
std::optional<photonlib::EstimatedRobotPose> CurrentEstimatedPose;
frc::Pose3d TagPose;
// vars for cone/cube cam
photonlib::PhotonCamera Cam2 = photonlib::PhotonCamera("Cam2");
photonlib::PhotonPipelineResult PieceCamResult;
photonlib::PhotonTrackedTarget PieceCamTarget;

double PieceCamPitch;
double PieceCamYaw;
double PieceCamSkew;

#endif
#ifdef OldVision
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
}
#endif

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

  // frc::SmartDashboard::PutNumber("Old X", OldX);
  // frc::SmartDashboard::PutNumber("Old Y", OldY);

  // frc::SmartDashboard::PutNumber("True Cam X", (V_VIS_m_TagX * C_MeterToIn)); //The V_VIS_in_Tag values have processesing to them,
  // frc::SmartDashboard::PutNumber("True Cam Y", (V_VIS_m_TagY * C_MeterToIn));//    these are the same numbers but don't get reverted from error
  // code for apriltag vision
  OldStamp = NewStamp;
  TagCamResult = estimator.GetCamera().GetLatestResult(); // GetCamera() pulls the camresult from the estimator value
  NewStamp = estimator.GetCamera().GetLatestResult().GetTimestamp().value();
  VeVIS_b_TagHasTarget = TagCamResult.HasTargets();
  double timeBtwn = NewStamp - OldStamp;
  frc::SmartDashboard::PutNumber("Time Since vision cycle", timeBtwn);
  if (VeVIS_b_TagHasTarget) // we only do anything if we have a target, otherwise this makes terrible data
  {

    CurrentEstimatedPose = estimator.Update();
    if (CurrentEstimatedPose.has_value()) //make sure photon is ready before we yoink its data
    {
      TagPose = CurrentEstimatedPose.value().estimatedPose; // "pose" object which holds xyz and roll,pitch,yaw values
    };
    V_TagID = TagCamResult.GetBestTarget().GetFiducialId();
    V_VIS_m_TagX = TagPose.X().value();
    V_VIS_m_TagY = TagPose.Y().value();
    // V_VIS_m_TagY = TagPose.Y().value();
    V_Tagz = TagPose.Z().value();
    // V_TagRoll = TagPose.Rotation().X().value();
    // V_TagPitch = TagPose.Rotation().Y().value();
    V_TagYaw = Filter_FirstOrderLag(TagPose.Rotation().Z().value(), V_TagYaw, K_TagYawFilter);

    OldX = V_VIS_in_TagX; // save last scans values to the old coords
    OldY = V_VIS_in_TagY;
    V_VIS_in_TagX = V_VIS_m_TagX * C_MeterToIn; // photon outputs meters, odom wants inches
    V_VIS_in_TagY = V_VIS_m_TagY * C_MeterToIn;

    if (OldX != 0.0 && OldY != 0.0)
    {
      XSpeed = (V_VIS_in_TagX - OldX) / timeBtwn; // calculate the speed we would've had to go at to travel our two distances within the time between the scans
      YSpeed = (V_VIS_in_TagY - OldY) / timeBtwn;

      frc::SmartDashboard::PutNumber("X speed", XSpeed);
      frc::SmartDashboard::PutNumber("Y speed", YSpeed);
    }
    else
    {
      XSpeed = 0.0;
      YSpeed = 0.0;
    }
    if ((fabs(XSpeed) < 40.0 && fabs(YSpeed) < 40.0)) // TBD:  make those 40s into constants, theyre calculated by converting the 12 ft/s of the robot to inches/.25s
    {

      OdometryInitToArgs(V_VIS_in_TagY, V_VIS_in_TagX); // if we are within a reasonable speed, update the odom
    }
    else
    {
      V_VIS_in_TagX = OldX; // if the speed isn't reasonable then revert to last scans values
      V_VIS_in_TagY = OldY;
    }
  }
  else
  {

    // zero values when no target
    V_TagID = 0;

    V_VIS_m_TagX = 0.0;
    V_VIS_m_TagY = 0.0;
    V_Tagz = 0.0;
    V_TagRoll = 0.0;
    V_TagPitch = 0.0;
    V_TagYaw = 0.0;
  }

  Cam2.SetDriverMode(true);

}

#endif