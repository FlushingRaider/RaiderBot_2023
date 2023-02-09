/*
  Manipulator.hpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift STATE machine? another government scam smh -chloe
 */

extern double       VeMan_Cnt_MoterCommandA;
extern double       VeMAN_Cnt_MoterCommandB;
extern T_Man_State VeMAN_Cnt_Man_state;
extern double VeMAN_Cnt_MoterTestPowerCmndA;
extern double VeMAN_Cnt_MoterTestPowerCmndB;
extern double VaMAN_v_MotorMaxCurrentA[E_Lift_State_Sz];
extern double VaMAN_v_MotorMaxCurrentB[E_Lift_State_Sz];
extern bool   VeMAN_b_WaitingForDriverINS;
extern bool   VeMAN_b_ArmInitialized;

void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
                         rev::SparkMaxPIDController m_liftpidXD);
 
void ManipulatorMoterConfigsInit(rev::SparkMaxPIDController m_liftpidYD,
                          rev::SparkMaxPIDController m_liftpidXD);

void ManipulatorControlInit();

void Manipulator_Control_ManualOverride(double              *LeMAN_Cmd_CommandA,
                                 double              *LeMAN_Cmd_CommandB,
                                 double               LeMAN_v_MotorCurrentOutA,
                                 double               LeMAN_v_MotorCurrentOutB,
                                 T_Manipulator_CmndDirection  LeMAN_Cmd_DriverMANDirection,
                                 bool                 LeMAN_b_LimitDetectedA,
                                 bool                 LeMAN_b_LimitDetectedB);

T_Man_State Lift_Control_Dictator(bool                LeLFT_b_AutoClimbButton,
                                   bool                L_driver_auto_climb_pause,
                                   T_Manipulator_CmndDirection LeMAN_Cmd_DriverMANDirection,
                                   double              L_game_time,
                                   T_Man_State        LeMAN_Cnt_CurrentState,
                                   double              LeLFT_In_MeasuredPositionYD,
                                   double              LeLFT_In_MeasuredPositionXD,
                                   double             *LeMAN_Cmd_CommandA,
                                   double             *LeMAN_Cmd_CommandB,
                                   double             *LeLFT_Pct_CommandPwrYD,
                                   double             *LeLFT_Pct_CommandPwrXD,
                                   bool                LeMAN_b_LimitDetectedA,
                                   bool                LeMAN_b_LimitDetectedB,
                                   double              LeLEFT_Deg_GyroAngleYaws,
                                   double              LeMAN_v_MotorCurrentOutA,
                                   double              LeMAN_v_MotorCurrentOutB,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD);