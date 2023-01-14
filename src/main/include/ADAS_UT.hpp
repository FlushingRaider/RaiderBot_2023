/*
  ADAS_UT.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS Upper Targeting

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_UT_UpperTarget V_ADAS_UT_State;
void ADAS_UT_Reset(void);
void ADAS_UT_ConfigsCal(void);
void ADAS_UT_ConfigsInit(void);

T_ADAS_ActiveFeature ADAS_UT_Main(double               *L_Pct_FwdRev,
                                  double               *L_Pct_Strafe,
                                  double               *L_Pct_Rotate,
                                  double               *L_RPM_Launcher,
                                  double               *L_Pct_Intake,
                                  double               *L_Pct_Elevator,
                                  bool                 *L_CameraUpperLightCmndOn,
                                  bool                 *L_CameraLowerLightCmndOn,
                                  bool                 *L_SD_RobotOriented,
                                  bool                 *L_VisionTargetingRequest,
                                   bool                 L_VisionTopTargetAquired,
                                  double                L_TopTargetYawDegrees,
                                  double                L_VisionTopTargetDistanceMeters,
                                  T_RobotState          L_RobotState,
                                  double                L_LauncherRPM_Measured,
                                  bool                  L_BallDetected,
                                  bool                  L_DriverRequestElevatorUp,
                                  bool                  L_DriverRequestElevatorDwn,
                                  bool                  L_DriverRequestIntake);