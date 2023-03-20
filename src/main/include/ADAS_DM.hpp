/*
  ADAS_DM.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS Upper Targeting

  Changes:
  2022-02-25 -> Beta
 */

extern double V_ADAS_DM_InitGyroAngle;
extern double V_ADAS_DM_Rotate180TargetAngle;
extern double VeADAS_t_DM_AutoMountDbTime;
extern TeADAS_DM_DriveOverStation VeADAS_e_DM_AutoMountState;


extern bool wantToStopX;
extern bool wantToStopY;

extern std::string V_MoveToTagStep;

void ADAS_DM_Reset(void);
void ADAS_DM_ConfigsCal(void);
void ADAS_DM_ConfigsInit(void);

bool ADAS_DM_Stop(double *L_Pct_FwdRev,
                  double *L_Pct_Strafe,
                  double *L_Pct_Rotate,
                  bool   *L_SD_RobotOriented,
                  double  LeADAS_t_StopTime);

bool ADAS_DM_DriveStraight(double *L_Pct_FwdRev,
                           double *L_Pct_Strafe,
                           double *L_Pct_Rotate,
                           bool *L_SD_RobotOriented);

bool ADAS_DM_DriveRevStraight(double *L_Pct_FwdRev,
                              double *L_Pct_Strafe,
                              double *L_Pct_Rotate,
                              bool   *L_SD_RobotOriented,
                              bool    LeADAS_b_CompletePrev);

bool ADAS_DM_Rotate180(double *L_Pct_FwdRev,
                       double *L_Pct_Strafe,
                       double *L_Pct_Rotate,
                       double *L_RPM_Launcher,
                       double *L_Pct_Intake,
                       double *L_Pct_Elevator,
                       bool *L_CameraUpperLightCmndOn,
                       bool *L_CameraLowerLightCmndOn,
                       bool *L_SD_RobotOriented,
                       double L_Deg_GyroAngleDeg);

bool ADAS_DM_RotateTo0(double *L_Pct_FwdRev,
                       double *L_Pct_Strafe,
                       double *L_Pct_Rotate,
                       double *L_RPM_Launcher,
                       double *L_Pct_Intake,
                       double *L_Pct_Elevator,
                       bool *L_CameraUpperLightCmndOn,
                       bool *L_CameraLowerLightCmndOn,
                       bool *L_SD_RobotOriented,
                       double L_Deg_GyroAngleDeg,
                       double L_InitGyroAngle);

bool ADAS_DM_PathFollower(double *L_Pct_FwdRev,
                          double *L_Pct_Strafe,
                          double *L_Pct_Rotate,
                          bool *L_SD_RobotOriented,
                          double L_L_X_FieldPos,
                          double L_L_Y_FieldPos,
                          double L_Deg_GyroAngleDeg,
                          int L_i_PathNum,
                          std::string VeADAS_Str_AutoPathName);

bool ADAS_DM_FieldOrientRotate(double *L_Pct_FwdRev,
                               double *L_Pct_Strafe,
                               double *L_Pct_Rotate,
                               double *L_RPM_Launcher,
                               double *L_Pct_Intake,
                               double *L_Pct_Elevator,
                               bool *L_CameraUpperLightCmndOn,
                               bool *L_CameraLowerLightCmndOn,
                               bool *L_SD_RobotOriented,
                               double L_Deg_GyroAngleDeg,
                               double L_Deg_GyroAngleTarget);

bool ADAS_DM_MoveToTag(double *L_Pct_FwdRev,
                       double *L_Pct_Strafe,
                       double *L_Pct_Rotate,
                       bool L_OdomCentered,
                       int L_TagID,
                       double L_OdometryX,
                       double L_OdometryY,
                       bool *L_VisionTargetingRequest,
                       double L_VisionTopTargetAquired,
                       double L_TagYawDegrees,
                       frc::DriverStation::Alliance LeLC_e_AllianceColor,
                       bool L_CubeAlignCmd,
                       bool L_ConeAlignCmd);

bool ADAS_DM_AutoBalance(double *L_Pct_FwdRev,
                         double *L_Pct_Strafe,
                         double *L_Pct_Rotate,
                         bool *L_SD_RobotOriented,
                         bool *LeADAS_b_X_Mode,
                         double LeADAS_Deg_GyroRoll);

bool ADAS_DM_DriveOntoStation(double *L_Pct_FwdRev,
                              double *L_Pct_Strafe,
                              double *L_Pct_Rotate,
                              bool   *L_SD_RobotOriented,
                              bool   *LeADAS_b_X_Mode,
                              double  LeADAS_Deg_GyroRoll);

bool ADAS_DM_MountStation(double *L_Pct_FwdRev,
                          double *L_Pct_Strafe,
                          double *L_Pct_Rotate,
                          bool   *L_SD_RobotOriented,
                          bool   *LeADAS_b_X_Mode,
                          double  LeADAS_Deg_GyroRoll);

bool MoveWithOffsetTag(double *L_Pct_FwdRev,
                       double *L_Pct_Strafe,
                       double *L_Pct_Rotate,
                       bool L_OdomCentered,
                       double L_TagYawDegrees,
                       double L_OdomOffsetX,
                       double L_OdomOffsetY,
                       double L_RequestedOffsetX,
                       double L_RequestedOffsetY);

bool MoveWithGlobalCoords(double *L_Pct_FwdRev,
                          double *L_Pct_Strafe,
                          double *L_Pct_Rotate,
                          bool L_OdomCentered,
                          double L_TagYawDegrees,
                          double L_CurrentOdomX,
                          double L_CurrentOdomY,
                          double L_RequestedCoordX,
                          double L_RequestedCoordY);

bool ADAS_DM_DriveStraightFar(double *L_Pct_FwdRev,
                              double *L_Pct_Strafe,
                              double *L_Pct_Rotate,
                              bool   *L_SD_RobotOriented);