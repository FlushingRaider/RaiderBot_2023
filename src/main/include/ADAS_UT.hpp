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

T_ADAS_ActiveFeature ADAS_UT_Main(double *L_Pct_FwdRev,
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
                                  frc::DriverStation::Alliance LeLC_e_AllianceColor);