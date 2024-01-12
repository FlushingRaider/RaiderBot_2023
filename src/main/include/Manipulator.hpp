/*
  Manipulator.hpp

   Created on: Feb 01, 2022
   Author: 5561

   The lift control state machine. This controls the robat to move the x and y hooks. It automously controls the robot to climb

   lift STATE machine? another government scam smh -chloe
 */

extern TeMAN_MotorControl      VsMAN_s_Motors;
extern TsMAN_Sensor            VsMAN_s_Sensors;
extern TeMAN_ManipulatorStates VeMAN_e_AttndState;
extern TeMAN_ManipulatorStates VeMAN_e_CmndState;
extern TeMAN_MotorControl      VsMAN_s_MotorsTest;
extern TeMAN_MotorControl      VsMAN_s_MotorsTemp;
extern double VaMAN_In_LinearSlideError;

void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_ArmPivotPID,
                                rev::SparkMaxPIDController m_WristPID,
                                rev::SparkMaxPIDController m_GripperPID);
 
void ManipulatorMotorConfigsInit(rev::SparkMaxPIDController m_ArmPivotPID,
                                 rev::SparkMaxPIDController m_WristPID,
                                 rev::SparkMaxPIDController m_GripperPID);

void ManipulatorControlInit();

void ManipulatorControlManualOverride(RobotUserInput *LsCONT_s_DriverInput);

void ManipulatorControlMain(TeMAN_ManipulatorStates LeMAN_e_SchedState,
                            bool                    LeMAN_b_TestPowerOverride,
                            bool                    LeADAS_b_MAN_DropObjectSlow,
                            bool                    LeADAS_b_MAN_DropObjectFast);