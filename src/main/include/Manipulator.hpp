/*
  Manipulator.hpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift STATE machine? another government scam smh -chloe
 */

extern bool   VeMAN_b_WaitingForDriverINS;
extern bool   VeMAN_b_ArmInitialized;
extern TeMAN_MotorControl VsMAN_s_Motors;
extern TsMAN_Sensor       VsMAN_s_Sensors;

void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_liftpidYD,
                         rev::SparkMaxPIDController m_liftpidXD);
 
void ManipulatorMotorConfigsInit(rev::SparkMaxPIDController m_ArmPivotPID,
                                 rev::SparkMaxPIDController m_WristPID,
                                 rev::SparkMaxPIDController m_GripperPID,
                                 rev::SparkMaxPIDController m_IntakeRollersPID);

void LiftControlInit();

void ManipulatorControlManualOverride(RobotUserInput *LsCONT_s_DriverInput);

TeMAN_ManipulatorStates ManipulatorControlDictator(bool                LeMAN_b_AutoManipulateButton,
                                   bool                L_driver_auto_climb_pause,
                                   TeLFT_e_LiftCmndDirection LeLFT_Cmd_DriverLiftDirection,
                                   double              L_game_time,
                                   TeMAN_ManipulatorStates        LeMAN_Cnt_CurrentState,
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