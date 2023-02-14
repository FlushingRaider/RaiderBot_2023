/*
  Manipulator.hpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift STATE machine? another government scam smh -chloe
 */

extern double       VeMan_Cnt_MoterCommandTurret;
extern double       VeMan_Cnt_MoterCommandArmPivot;
extern double       VeMAN_Cnt_MoterCommandLinearSlide;
extern double       VeMAN_Cnt_MoterCommandWrist;
extern double       VeMAN_Cnt_MoterCommandClaw;
extern double       VeMAN_Cnt_MoterCommandIntake;
extern T_Man_DoesStuffMaybe VeMAN_Cnt_Man_state;
extern double VeMAN_Cnt_MoterTestPowerCmndTurret;
extern double VeMAN_Cnt_MoterTestPowerCmndArmPivot;
extern double VeMAN_Cnt_MoterTestPowerCmndClawlevator;
extern double VeMAN_Cnt_MoterTestPowerCmndWrist;
extern double VeMAN_Cnt_MoterTestPowerCmndClaw;
extern double VeMAN_Cnt_MoterTestPowerCmndIntake;
extern double VaMAN_v_MotorMaxCurrentTurret[E_Man_State_Sz];
extern double VaMAN_v_MotorMaxCurrentArmPivot[E_Man_State_Sz];
extern double VaMAN_v_MotorMaxCurrentLinearSlide[E_Man_State_Sz];
extern double VaMAN_v_MotorMaxCurrentWrist[E_Man_State_Sz];
extern double VaMAN_v_MotorMaxCurrentClaw[E_Man_State_Sz];
extern double VaMAN_v_MotorMaxCurrentIntake[E_Man_State_Sz];
extern bool   VeMAN_b_WaitingForDriverINS;
extern bool   VeMAN_b_ArmInitialized;

void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
                         rev::SparkMaxPIDController m_liftpidXD);
 
void ManipulatorMoterConfigsInit(rev::SparkMaxPIDController m_liftpidYD,
                          rev::SparkMaxPIDController m_liftpidXD);

void ManipulatorControlInit();

void Manipulator_Control_ManualOverride(double              *LeMAN_Cmd_CommandTurret,
                                 double              *LeMAN_Cmd_CommandArmPivot,
                                 double               LeMAN_v_MotorCurrentOutTurret,
                                 double               LeMAN_v_MotorCurrentOutArmPivot,
                                 double       LeMAN_v_MotorCurrentOutLinearSlide,
                                 double       LeMAN_v_MotorCurrentOutWrist,
                                 double       LeMAN_v_MotorCurrentOutClaw,
                                 double       LeMAN_v_MotorCurrentOutIntake,
                                 T_Manipulator_CmndDirection  LeMAN_Cmd_DriverMANDirection,
                                 bool                 LeMAN_b_LimitDetectedTurret,
                                 bool                 LeMAN_b_LimitDetectedArmPivot);

T_Man_DoesStuffMaybe ManipulatorControlDictator(bool                LeMAN_b_AutoManipulateButton,
                                   bool                L_driver_auto_climb_pause,
                                   T_Manipulator_CmndDirection LeMAN_Cmd_DriverMANDirection,
                                   double              L_game_time,
                                   T_Man_DoesStuffMaybe        LeMAN_Cnt_CurrentState,
                                   double              LeMAN_Deg_MeasuredAngleTurret,
                                   double              LeMAN_Deg_MeasuredAngleArmPivot,
                                   double             *LeMAN_Cmd_CommandTurret,
                                   double             *LeMAN_Cmd_CommandArmPivot,
                                   double             *LeLFT_Pct_CommandPwrYD,
                                   double             *LeLFT_Pct_CommandPwrXD,
                                   bool                LeMAN_b_LimitDetectedTurret,
                                   bool                LeMAN_b_LimitDetectedArmPivot,
                                   double              LeLEFT_Deg_GyroAngleYaws,
                                   double              LeMAN_v_MotorCurrentOutTurret,
                                   double              LeMAN_v_MotorCurrentOutArmPivot,
                                   double              LeMAN_v_MotorCurrentOutLinearSlide,
                                   double              LeMAN_v_MotorCurrentOutWrist,
                                   double              LeMAN_v_MotorCurrentOutClaw,
                                   double              LeMAN_v_MotorCurrentOutIntake,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD);