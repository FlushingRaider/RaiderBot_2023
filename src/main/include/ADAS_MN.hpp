
extern TeMAN_ManipulatorStates VeADAS_e_MAN_SchedState;
extern bool                    VeADAS_b_MAN_DropObjectSlow;
extern bool                    VeADAS_b_MAN_DropObjectFast;

void ADAS_MN_ConfigsInit();

void ADAS_MN_ConfigsCal();

void ADAS_MN_Reset(void);

bool ADAS_MN_Main(T_RobotState                  LeADAS_e_RobotState,
                  T_ADAS_ActiveFeature          LeADAS_e_ActiveFeature,
                  TeADAS_AutonManipulatorStates LeADAS_e_MAN_ReqAction);