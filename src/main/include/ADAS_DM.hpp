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

void ADAS_DM_Reset(void);
void ADAS_DM_ConfigsCal(void);
void ADAS_DM_ConfigsInit(void);

bool ADAS_DM_BlindShot(double       *L_Pct_FwdRev,
                       double       *L_Pct_Strafe,
                       double       *L_Pct_Rotate,
                       double       *L_RPM_Launcher,
                       double       *L_Pct_Intake,
                       double       *L_Pct_Elevator,
                       bool         *L_CameraUpperLightCmndOn,
                       bool         *L_CameraLowerLightCmndOn,
                       bool         *L_SD_RobotOriented);

bool ADAS_DM_ReverseAndIntake(double     *L_Pct_FwdRev,
                              double     *L_Pct_Strafe,
                              double     *L_Pct_Rotate,
                              double     *L_RPM_Launcher,
                              double     *L_Pct_Intake,
                              double     *L_Pct_Elevator,
                              bool       *L_CameraUpperLightCmndOn,
                              bool       *L_CameraLowerLightCmndOn,
                              bool       *L_SD_RobotOriented,
                              double      L_DriveTime);

bool ADAS_DM_DriveStraight(double     *L_Pct_FwdRev,
                           double     *L_Pct_Strafe,
                           double     *L_Pct_Rotate,
                           double     *L_RPM_Launcher,
                           double     *L_Pct_Intake,
                           double     *L_Pct_Elevator,
                           bool       *L_CameraUpperLightCmndOn,
                           bool       *L_CameraLowerLightCmndOn,
                           bool       *L_SD_RobotOriented);

bool ADAS_DM_Rotate180(double     *L_Pct_FwdRev,
                       double     *L_Pct_Strafe,
                       double     *L_Pct_Rotate,
                       double     *L_RPM_Launcher,
                       double     *L_Pct_Intake,
                       double     *L_Pct_Elevator,
                       bool       *L_CameraUpperLightCmndOn,
                       bool       *L_CameraLowerLightCmndOn,
                       bool       *L_SD_RobotOriented,
                       double      L_Deg_GyroAngleDeg);

bool ADAS_DM_RotateTo0(double     *L_Pct_FwdRev,
                       double     *L_Pct_Strafe,
                       double     *L_Pct_Rotate,
                       double     *L_RPM_Launcher,
                       double     *L_Pct_Intake,
                       double     *L_Pct_Elevator,
                       bool       *L_CameraUpperLightCmndOn,
                       bool       *L_CameraLowerLightCmndOn,
                       bool       *L_SD_RobotOriented,
                       double      L_Deg_GyroAngleDeg,
                       double      L_InitGyroAngle);

bool ADAS_DM_PathFollower(double *L_Pct_FwdRev,
                          double *L_Pct_Strafe,
                          double *L_Pct_Rotate,
                          double *L_Pct_Intake,
                          bool   *L_SD_RobotOriented,
                          double  L_L_X_FieldPos,
                          double  L_L_Y_FieldPos,
                          double  L_Deg_GyroAngleDeg,
                          int     L_i_PathNum,
                          std::string V_ADAS_Auto_PathName);

bool ADAS_DM_FieldOrientRotate(double     *L_Pct_FwdRev,
                               double     *L_Pct_Strafe,
                               double     *L_Pct_Rotate,
                               double     *L_RPM_Launcher,
                               double     *L_Pct_Intake,
                               double     *L_Pct_Elevator,
                               bool       *L_CameraUpperLightCmndOn,
                               bool       *L_CameraLowerLightCmndOn,
                               bool       *L_SD_RobotOriented,
                               double      L_Deg_GyroAngleDeg,
                               double      L_Deg_GyroAngleTarget);

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
                          bool   *L_SD_RobotOriented,
                          bool   *LeADAS_b_X_Mode,
                          double  LeADAS_Deg_GyroRoll);