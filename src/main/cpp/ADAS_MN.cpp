/*
  ADAS_MN.cpp

  Created on: Feb 21, 2023
  Author: Jay L

  ADAS (Advanced Driver-Assistance Systems) Manipulator
  Contains the logic and code used to schedule states for the Manipulator:
    -Schedules states from a non-linear state machine to be executed until "called" state is reached

  Changes:
  2-21-2023 -> Alpha
 */

#ifdef HOMOSEXUAL

#ifdef Its 1 AM and J hasnt slept but also managed to make 0 progress and probably started doing the wrong thing because hes tired
/******************************************************************************
 * Function:      ADAS_MN_SwitchCase
 * Author: Jay L
 * Description: Switches scheduled states based on Cont. Input
 ******************************************************************************/
T_ADAS_MN_Manipulator ADAS_MN_MoveToTag(double *L_Pct_FwdRev,
                                        double *L_Pct_Strafe,
                                        double *L_Pct_Rotate,
                                        bool L_OdomCentered,
                                        int L_TagID,
                                        double L_OdometryX,
                                        double L_OdometryY,
                                        bool *L_VisionTargetingRequest,
                                        double L_VisionTopTargetAquired,
                                        double L_TagYawDegrees,
                                        frc::DriverStation::Alliance LeLC_e_AllianceColor)
{

  T_ADAS_MN_Manipulator L_ADAS_MN_State = E_ADAS_MN_MoveToTag;

  *L_Pct_FwdRev = 0;
  *L_Pct_Strafe = 0;
  /* Next, let's set all the other items we aren't trying to control to off: */
  double L_ChosenX = 0;
  double L_ChosenY = 0;
  double L_ErrorCalcYaw = 0;
  int L_ClosestTag;

  // all 3 tags are directly across from each other

  // coord of tag ID 1 and 8
  double L_Tag1Y = 1.071626;
  // coord of tag ID 2 and 7
  double L_Tag2Y = 2.748026;
  // coord of tag ID 3 and 6
  double L_Tag3Y = 4.424426;
  double L_TagXred = 15.513558;
  double L_TagXblue = 1.02743;

  // double L_Tag1YError;
  // double L_Tag2YError;
  // double L_Tag3YError;

  V_ADAS_MN_DebounceTime += C_ExeTime;
  if (L_OdomCentered) // don't do any of this if we haven't centered our Odometry based on the tag
  {
    // pick the right side of tags to look at for our alliance
    if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
    {
      L_ChosenX = L_TagXred; // all 3 tags on the red alliance have the same X
      if (L_TagID == 1)
      {
        L_ChosenY = L_Tag1Y;
      }
      else if (L_TagID == 2)
      {
        L_ChosenY = L_Tag2Y;
      }
      else if (L_TagID == 3)
      {
        L_ChosenY = L_Tag3Y;
      }
    }
    else
    {
      L_ChosenX = L_TagXblue; // x coord of all blue tags
      if (L_TagID == 8)
      {
        L_ChosenY = L_Tag1Y;
      }
      else if (L_TagID == 7)
      {
        L_ChosenY = L_Tag2Y;
      }
      else if (L_TagID == 6)
      {
        L_ChosenY = L_Tag3Y;
      }
    }

    // find the closest tag and our error to it:
    if (L_VisionTopTargetAquired)
    {

      // // check if Error 1 is the largest
      // if (L_Tag1YError >= L_Tag2YError && L_Tag1YError >= L_Tag3YError)
      // {
      //   L_ChosenY = L_Tag1Y;
      // }
      // // check if Error 2 is the largest number
      // else if (L_Tag2YError >= L_Tag1YError && L_Tag2YError >= L_Tag3YError)
      // {
      //   L_ChosenY = L_Tag2Y;
      // }
      // // if neither n1 nor n2 are the largest, Error 3 is the largest
      // else
      // {
      //   L_ChosenY = L_Tag3Y;
      // }

      L_ErrorCalcYaw = 0.0 - L_TagYawDegrees;
    }
    if (V_ADAS_MN_DebounceTime <= KV_ADAS_MN_TurretMinDeltaOL) // make sure we're still in the time we've given ourselves
    {
      if (L_ErrorCalcYaw > 0 || L_ErrorCalcYaw < 0)
      {
        *L_Pct_Rotate = DesiredAutoRotateSpeed(L_ErrorCalcYaw);
      }
      if (L_OdometryX < L_ChosenX) // TODO: add deadband
      {
        *L_Pct_FwdRev = 0.2;
      }
      else if (L_OdometryX > L_ChosenX)
      {
        *L_Pct_FwdRev = -0.2;
      }
      else
      {
        *L_Pct_FwdRev = 0.0;
      }
      if (L_OdometryY < L_ChosenY) // TODO: add deadband
      {
        *L_Pct_Strafe = 0.2;
      }
      else if (L_OdometryY > L_ChosenY)
      {
        *L_Pct_Strafe = -0.2;
      }
      else
      {
        *L_Pct_Strafe = 0.0;
      }
    }
    else if (V_ADAS_MN_DebounceTime >= 0.3)
    {
      V_ADAS_MN_DebounceTime = 0;
      *L_Pct_FwdRev = 0;
      *L_Pct_Strafe = 0;
    }
  }
  return (L_ADAS_MN_State);
}
#endif

/******************************************************************************
 * Function:     ADAS_MN_Main
 *
 * Description:  Manages and controls the upper targeting (UT) and spinning up of
 *               the launcher.
 ******************************************************************************/
bool ADAS_MN_Main(double *L_Pct_FwdRev,
                  double *L_Pct_Strafe,
                  double *L_Pct_Rotate,
                  double *L_Pct_Intake,
                  bool *L_VisionTargetingRequest,
                  bool L_VisionTopTargetAquired,
                  T_RobotState L_RobotState,
                  int L_TagID,
                  bool L_OdomCentered,
                  double L_OdometryX,
                  double L_OdometryY,
                  double L_TagYawDegrees,
                  frc::DriverStation::Alliance LeLC_e_AllianceColor)
{
  bool L_ADAS_MN_Complete = false;

  switch (V_ADAS_MN_State)
  {
  case E_ADAS_MN_Disabled:
  case E_ADAS_MN_CameraLightOn:

#ifdef unused
    V_ADAS_MN_State = ADAS_MN_CameraLightOn(L_Pct_FwdRev,
                                            L_Pct_Strafe,
                                            L_Pct_Rotate,
                                            L_RPM_Launcher,
                                            L_Pct_Intake,
                                            L_Pct_Elevator,
                                            L_CameraUpperLightCmndOn,
                                            L_CameraLowerLightCmndOn,
                                            L_SD_RobotOriented,
                                            L_VisionTargetingRequest);
    break;
#endif

  case E_ADAS_MN_AutoCenter:
#ifdef unused
    V_ADAS_MN_State = ADAS_MN_AutoCenter(L_Pct_FwdRev,
                                         L_Pct_Strafe,
                                         L_Pct_Rotate,
                                         L_RPM_Launcher,
                                         L_Pct_Intake,
                                         L_Pct_Elevator,
                                         L_CameraUpperLightCmndOn,
                                         L_CameraLowerLightCmndOn,
                                         L_SD_RobotOriented,
                                         L_VisionTargetingRequest,
                                         L_VisionTopTargetAquired,
                                         L_TopTargetYawDegrees);
    break;
#endif

  case E_ADAS_MN_LauncherSpeed:
#ifdef unused
    V_ADAS_MN_State = ADAS_MN_LauncherSpeed(L_Pct_FwdRev,
                                            L_Pct_Strafe,
                                            L_Pct_Rotate,
                                            L_RPM_Launcher,
                                            L_Pct_Intake,
                                            L_Pct_Elevator,
                                            L_CameraUpperLightCmndOn,
                                            L_CameraLowerLightCmndOn,
                                            L_SD_RobotOriented,
                                            L_VisionTargetingRequest,
                                            L_VisionTopTargetAquired,
                                            L_VisionTopTargetDistanceMeters);
    break;
#endif

  case E_ADAS_MN_ElevatorControl:
#ifdef unused
    V_ADAS_MN_State = ADAS_MN_ElevatorControl(L_Pct_FwdRev,
                                              L_Pct_Strafe,
                                              L_Pct_Rotate,
                                              L_RPM_Launcher,
                                              L_Pct_Intake,
                                              L_Pct_Elevator,
                                              L_CameraUpperLightCmndOn,
                                              L_CameraLowerLightCmndOn,
                                              L_SD_RobotOriented,
                                              L_VisionTargetingRequest,
                                              L_RobotState,
                                              L_LauncherRPM_Measured,
                                              L_BallDetectedUpper,
                                              L_DriverRequestElevatorUp,
                                              L_DriverRequestElevatorDwn,
                                              L_DriverRequestIntake);
    break;
#endif

  case E_ADAS_MN_MoveToTag:
    V_ADAS_MN_State = ADAS_MN_MoveToTag(L_Pct_FwdRev,
                                        L_Pct_Strafe,
                                        L_Pct_Rotate,
                                        L_OdomCentered,
                                        L_TagID,
                                        L_OdometryX,
                                        L_OdometryY,
                                        L_VisionTargetingRequest,
                                        L_VisionTopTargetAquired,
                                        L_TagYawDegrees,
                                        LeLC_e_AllianceColor);
    break;
  }

  if (V_ADAS_MN_State == E_ADAS_MN_Disabled)
  {
    ADAS_MN_Reset();
    L_ADAS_MN_Complete = true;
  }

  return (L_ADAS_MN_Complete);
}
#endif