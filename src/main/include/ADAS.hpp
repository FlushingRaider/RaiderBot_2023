/*
  ADAS.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_ActiveFeature V_ADAS_ActiveFeature;

extern double               V_ADAS_Pct_SD_FwdRev;
extern double               V_ADAS_Pct_SD_Strafe;
extern double               V_ADAS_Pct_SD_Rotate;
extern double               V_ADAS_RPM_BH_Launcher;
extern double               V_ADAS_Pct_BH_Intake;
extern double               V_ADAS_Pct_BH_Elevator;
extern bool                 V_ADAS_CameraUpperLightCmndOn;
extern bool                 V_ADAS_CameraLowerLightCmndOn;
extern bool                 V_ADAS_SD_RobotOriented;
extern bool                 V_ADAS_Vision_RequestedTargeting;

void ADAS_Main_Reset(void);
void ADAS_Main_Init(void);
void ADAS_DetermineMode(void);


T_ADAS_ActiveFeature ADAS_ControlMain(double              *L_Pct_FwdRev,
                                      double               *L_Pct_Strafe,
                                      double               *L_Pct_Rotate,
                                      double               *L_RPM_Launcher,
                                      double               *L_Pct_Intake,
                                      double               *L_Pct_Elevator,
                                      bool                 *L_CameraUpperLightCmndOn,
                                      bool                 *L_CameraLowerLightCmndOn,
                                      bool                 *L_SD_RobotOriented,
                                      bool                 *L_ADAS_Vision_RequestedTargeting,
                                      bool                  L_Driver1_JoystickActive,
                                      bool                  L_Driver_stops_shooter,
                                      bool                  L_Driver_SwerveGoalAutoCenter,
                                      bool                  L_Driver_AutoIntake,
                                      double                L_Deg_GyroAngleDeg,
                                      double                L_L_X_FieldPos,
                                      double                L_L_Y_FieldPos,
                                      bool                  L_VisionTopTargetAquired,
                                      double                L_TopTargetYawDegrees,
                                      double                L_VisionTopTargetDistanceMeters,
                                      bool                  L_VisionBottomTargetAquired,
                                      double                L_VisionBottomYaw,
                                      double                L_VisionBottomTargetDistanceMeters,
                                      T_RobotState          L_RobotState,
                                      double                L_LauncherRPM_Measured,
                                      bool                  L_BallDetectedUpper,
                                      bool                  L_BallDetectedLower,
                                      bool                  L_DriverRequestElevatorUp,
                                      bool                  L_DriverRequestElevatorDwn,
                                      bool                  L_DriverRequestIntake,
                                      T_ADAS_ActiveFeature  L_ADAS_ActiveFeature);