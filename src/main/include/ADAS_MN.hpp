
extern TeMAN_ManipulatorStates VeADAS_e_MAN_SchedState;
extern bool                    VeADAS_b_MAN_DropObject;

void ADAS_MN_ConfigsInit();

void ADAS_MN_ConfigsCal();

void ADAS_MN_Reset(void);

bool ADAS_MN_Main(T_RobotState         L_RobotState,
                  T_ADAS_ActiveFeature LeADAS_e_ActiveFeature);