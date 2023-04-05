/*
  ADAS_DM.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS Drive Management

  Changes:
  2022-02-25 -> Beta
 */

extern double V_ADAS_DM_InitGyroAngle;
extern double VeADAS_t_DM_AutoMountDbTime;
extern TeADAS_DM_DriveOverStation VeADAS_e_DM_AutoMountState;


extern bool wantToStopX;
extern bool wantToStopY;

extern std::string V_MoveToTagStep;

void ADAS_DM_Reset(void);
void ADAS_DM_ConfigsCal(void);
void ADAS_DM_ConfigsInit(void);

bool ADAS_DM_Stop(double *LeADAS_Pct_FwdRev,
                  double *LeADAS_Pct_Strafe,
                  double *LeADAS_Pct_Rotate,
                  bool   *LeADAS_b_SD_RobotOriented,
                  double  LeADAS_t_StopTime);

bool ADAS_DM_DriveStraight(double *LeADAS_Pct_FwdRev,
                           double *LeADAS_Pct_Strafe,
                           double *LeADAS_Pct_Rotate,
                           bool *LeADAS_b_SD_RobotOriented);

bool ADAS_DM_DriveRevStraight(double *LeADAS_Pct_FwdRev,
                              double *LeADAS_Pct_Strafe,
                              double *LeADAS_Pct_Rotate,
                              bool   *LeADAS_b_SD_RobotOriented,
                              bool    LeADAS_b_CompletePrev);

bool ADAS_DM_PathFollower(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          double *LeADAS_Deg_DesiredPose,
                          bool   *LeADAS_b_SD_RobotOriented,
                          double  LeADAS_l_X_FieldPos,
                          double  LeADAS_l_Y_FieldPos,
                          double  LeADAS_Deg_GyroAngle,
                          T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                          frc::DriverStation::Alliance LeLC_e_AllianceColor);

bool ADAS_DM_MoveToTag(double *LeADAS_Pct_FwdRev,
                       double *LeADAS_Pct_Strafe,
                       double *LeADAS_Pct_Rotate,
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

bool ADAS_DM_AutoBalance(double *LeADAS_Pct_FwdRev,
                         double *LeADAS_Pct_Strafe,
                         double *LeADAS_Pct_Rotate,
                         bool *LeADAS_b_SD_RobotOriented,
                         bool *LeADAS_b_X_Mode,
                         double LeADAS_Deg_GyroRoll);

bool ADAS_DM_DriveOntoStation(double *LeADAS_Pct_FwdRev,
                              double *LeADAS_Pct_Strafe,
                              double *LeADAS_Pct_Rotate,
                              bool   *LeADAS_b_SD_RobotOriented,
                              bool   *LeADAS_b_X_Mode,
                              double  LeADAS_Deg_GyroRoll);

bool ADAS_DM_MountStation(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          bool   *LeADAS_b_SD_RobotOriented,
                          bool   *LeADAS_b_X_Mode,
                          double  LeADAS_Deg_GyroRoll);

bool MoveWithOffsetTag(double *LeADAS_Pct_FwdRev,
                       double *LeADAS_Pct_Strafe,
                       double *LeADAS_Pct_Rotate,
                       bool L_OdomCentered,
                       double L_TagYawDegrees,
                       double L_OdomOffsetX,
                       double L_OdomOffsetY,
                       double L_RequestedOffsetX,
                       double L_RequestedOffsetY);

bool MoveWithGlobalCoords(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          bool L_OdomCentered,
                          double L_TagYawDegrees,
                          double L_CurrentOdomX,
                          double L_CurrentOdomY,
                          double L_RequestedCoordX,
                          double L_RequestedCoordY);

bool ADAS_DM_DriveStraightFar(double *LeADAS_Pct_FwdRev,
                              double *LeADAS_Pct_Strafe,
                              double *LeADAS_Pct_Rotate,
                              bool   *LeADAS_b_SD_RobotOriented);