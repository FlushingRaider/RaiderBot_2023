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

void LiftMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
                         rev::SparkMaxPIDController m_liftpidXD);
 
void LiftMotorConfigsInit(rev::SparkMaxPIDController m_liftpidYD,
                          rev::SparkMaxPIDController m_liftpidXD);

void LiftControlInit();

void Lift_Control_ManualOverride(double              *LeLFT_Cmd_CommandYD,
                                 double              *LeLFT_Cmd_CommandXD,
                                 double               LeLFT_v_MotorYDCurrentOut,
                                 double               LeLFT_v_MotorXDCurrentOut,
                                 TeLFT_e_LiftCmndDirection  LeLFT_Cmd_DriverLiftDirection,
                                 bool                 LeLFT_b_LimitDetectedYD,
                                 bool                 LeLFT_b_LimitDetectedXD);

T_Man_State Lift_Control_Dictator(bool                LeLFT_b_AutoClimbButton,
                                   bool                L_driver_auto_climb_pause,
                                   TeLFT_e_LiftCmndDirection LeLFT_Cmd_DriverLiftDirection,
                                   double              L_game_time,
                                   T_Man_State        LeLFT_Cnt_CurrentState,
                                   double              LeLFT_In_MeasuredPositionYD,
                                   double              LeLFT_In_MeasuredPositionXD,
                                   double             *LeLFT_Cmd_CommandYD,
                                   double             *LeLFT_Cmd_CommandXD,
                                   double             *LeLFT_Pct_CommandPwrYD,
                                   double             *LeLFT_Pct_CommandPwrXD,
                                   bool                LeLFT_b_LimitDetectedYD,
                                   bool                LeLFT_b_LimitDetectedXD,
                                   double              LeLEFT_Deg_GyroAngleYaws,
                                   double              LeLFT_v_MotorYDCurrentOut,
                                   double              LeLFT_v_MotorXDCurrentOut,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD);