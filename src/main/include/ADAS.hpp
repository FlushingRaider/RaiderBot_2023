/*
  ADAS.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_ActiveFeature V_ADAS_ActiveFeature;

extern double V_ADAS_Pct_SD_FwdRev;
extern double V_ADAS_Pct_SD_Strafe;
extern double V_ADAS_Pct_SD_Rotate;
#ifdef unused
extern double V_ADAS_RPM_BH_Launcher;
#endif
extern double V_ADAS_Pct_BH_Intake;
extern double V_ADAS_Pct_BH_Elevator;
extern bool V_ADAS_CameraUpperLightCmndOn;
extern bool V_ADAS_CameraLowerLightCmndOn;
extern bool V_ADAS_SD_RobotOriented;
extern bool V_ADAS_Vision_RequestedTargeting;
extern bool VeADAS_b_X_Mode;

extern double VeADAS_in_OffsetRequestX;
extern double VeADAS_in_OffsetRequestY;

extern double VeADAS_in_GlobalRequestX;
extern double VeADAS_in_GlobalRequestY;

void ADAS_Main_Reset(void);
void ADAS_Main_Init(void);
void ADAS_DetermineMode(void);

T_ADAS_ActiveFeature ADAS_ControlMain(double *L_Pct_FwdRev,
                                      double *L_Pct_Strafe,
                                      double *L_Pct_Rotate,
                                      double *L_Pct_Intake,
                                      bool *L_SD_RobotOriented,
                                      bool *LeADAS_b_X_Mode,
                                      bool *L_VisionTargetingRequest,
                                      bool LeADAS_b_Driver1_JoystickActive,
                                      bool L_Driver_SwerveGoalAutoCenter,
                                      double L_Deg_GyroAngleDeg,
                                      double L_L_X_FieldPos,
                                      double L_L_Y_FieldPos,
                                      bool L_VisionTopTargetAquired,
                                      T_RobotState LeADAS_e_RobotState,
                                      T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                                      int L_TagID,
                                      bool L_OdomCentered,
                                      double L_TagYawDegrees,
                                      frc::DriverStation::Alliance LeLC_e_AllianceColor,
                                      double L_OdomOffsetX,
                                      double L_OdomOffsetY,
                                      double L_OdomGlobalRequestX,
                                      double L_OdomGlobalRequestY,
                                      double L_OdomOffsetRequestX,
                                      double L_OdomOffsetRequestY);