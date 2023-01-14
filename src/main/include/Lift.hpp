/*
  Lift.hpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift STATE machine? another government scam smh -chloe
 */

extern double       V_lift_command_YD;
extern double       V_lift_command_XD;
extern T_Lift_State V_Lift_state;
extern double V_LiftYD_TestPowerCmnd;
extern double V_LiftXD_TestPowerCmnd;
extern double V_LiftMotorYD_MaxCurrent[E_Lift_State_Sz];
extern double V_LiftMotorXD_MaxCurrent[E_Lift_State_Sz];
extern bool   V_Lift_WaitingForDriverINS;
extern bool   V_LiftInitialized;

void LiftMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
                         rev::SparkMaxPIDController m_liftpidXD);
 
void LiftMotorConfigsInit(rev::SparkMaxPIDController m_liftpidYD,
                          rev::SparkMaxPIDController m_liftpidXD);

void LiftControlInit();

void Lift_Control_ManualOverride(double              *L_lift_command_YD,
                                 double              *L_lift_command_XD,
                                 double               L_liftMotorYD_CurrentOut,
                                 double               L_liftMotorXD_CurrentOut,
                                 T_LiftCmndDirection  L_DriverLiftCmndDirection,
                                 bool                 L_YD_LimitDetected,
                                 bool                 L_XD_LimitDetected);

T_Lift_State Lift_Control_Dictator(bool                L_driver_auto_climb_button,
                                   bool                L_driver_auto_climb_pause,
                                   T_LiftCmndDirection L_DriverLiftCmndDirection,
                                   double              L_game_time,
                                   T_Lift_State        L_current_state,
                                   double              L_lift_measured_position_YD,
                                   double              L_lift_measured_position_XD,
                                   double             *L_lift_command_YD,
                                   double             *L_lift_command_XD,
                                   double             *L_Lift_CommandPwr_YD,
                                   double             *L_Lift_CommandPwr_XD,
                                   bool                L_YD_LimitDetected,
                                   bool                L_XD_LimitDetected,
                                   double              L_gyro_yawangledegrees,
                                   double              L_liftMotorYD_CurrentOut,
                                   double              L_liftMotorXD_CurrentOut,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftYD,
                                   rev::SparkMaxRelativeEncoder m_encoderLiftXD);