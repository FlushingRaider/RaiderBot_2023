
void ADAS_MN_ConfigsInit();

void ADAS_MN_ConfigsCal();

void ADAS_MN_Reset(void);

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
                  frc::DriverStation::Alliance LeLC_e_AllianceColor,
                  bool L_CubeAlignCmd,
                  bool L_ConeAlignCmd);