/*
  ADAS_BT.hpp

  Created on: Feb 26, 2022
  Author: Biggs

  ADAS Ball Targeting

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_BT_BallTarget V_ADAS_BT_State;
void ADAS_BT_Reset(void);
void ADAS_BT_ConfigsCal(void);
void ADAS_BT_ConfigsInit(void);

bool ADAS_BT_Main(double               *L_Pct_FwdRev,
                  double               *L_Pct_Strafe,
                  double               *L_Pct_Rotate,
                  double               *L_RPM_Launcher,
                  double               *L_Pct_Intake,
                  double               *L_Pct_Elevator,
                  bool                 *L_CameraUpperLightCmndOn,
                  bool                 *L_CameraLowerLightCmndOn,
                  bool                 *L_SD_RobotOriented,
                  bool                 *L_VisionTargetingRequest,
                  bool                  L_VisionBottomTargetAquired,
                  double                L_VisionBottomYaw,
                  double                L_VisionBottomTargetDistanceMeters,
                  T_RobotState          L_RobotState,
                  bool                  L_BallDetectedUpper,
                  bool                  L_BallDetectedLower);