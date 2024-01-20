#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired test state here: COMP (no test), BallHandlerTest, Manipulator_Test, DriveMotorTest, WheelAngleTest, ADAS_DM_Test
#define COMP
// Define the bot type: CompBot, PracticeBot
#define PracticeBot

// New PID calibration things
#define PID_Calibrate

#define NewVision // NewVision or OldVision
#define visionTest
// RoboRio controller execution time
const double C_ExeTime = 0.02;              // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms

// Amount of time for end game
const double C_End_game_time = 30;

// Numerical constants
const double C_RadtoDeg = 57.2957795;
const double C_Deg2Rad = 0.017453292519943295;
const double C_MeterToIn = 39.37008;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;

// CAN Device IDs:
static const int C_PDP_ID = 21;
static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID = 5, rearLeftDriveDeviceID = 6, rearRightSteerDeviceID = 7, rearRightDriveDeviceID = 8;
static const int KeMAN_i_LinearSlide = 10;
static const int KeMAN_i_ArmPivot = 11;
static const int KeMAN_i_Wrist = 12;
static const int KeMAN_i_Gripper = 13;
//static const int KeINT_i_IntakeRollers = 14;
static const int KeINT_i_IntakeArm = 15;
static const int KeGRY_i_Gyro = 16;
static const int KeEnc_i_WheelAngleFL = 17;
static const int KeEnc_i_WheelAngleFR = 18;
static const int KeEnc_i_WheelAngleRL = 19;
static const int KeEnc_i_WheelAngleRR = 20;
static const int KeINT_i_PCM = 22;

// Analog IDs:
static const int C_MagEncoderFL_ID = 2, C_MagEncoderFR_ID = 1, C_MagEncoderRL_ID = 3, C_MagEncoderRR_ID = 0;

// DIO IDs:


// PWM IDs:
static const int C_VanityLight_ID = 3;

// Vision Cals:

const double K_TagCordFilter = 0.08;
const double K_TagYawFilter = 0.08;

#ifdef PracticeBot
const double C_VisOffsetY = 1.0;
const double C_VisOffsetX = 2.0;
#endif

#ifdef CompBot
const double C_VisOffsetY = 3.125;
const double C_VisOffsetX = 13.5;
#endif
const double C_Tag1Y = 1.071626 * C_MeterToIn; // all these come in meters, we need them in inches to match odometry
                                               // coord of tag ID 2 and 7
const double C_Tag2Y = 2.748026 * C_MeterToIn;
// coord of tag ID 3 and 6
const double C_Tag3Y = 4.424426 * C_MeterToIn;
const double C_TagXred = 15.513558 * C_MeterToIn;
const double C_TagXblue = 1.02743 * C_MeterToIn;
const double C_TagAlignBasePower = 0.02;
const double C_TagScoreOffsetXCube = 34.0; // that little space our bumper is against to score cubes
const double C_TagScoreOffsetYCube = 0.0;
const double C_TagScoreOffsetXCone = 35.5; // that little space our bumper is against to score cubes
const double C_TagScoreOffsetYCone = 29.0;

const double K_MoveToTagMovementDeadbandX = 0.5;  // inches
const double K_MoveToTagMovementDeadbandY = 0.05; // inches

// const double K_MoveToTagRotationDeadband = 2.0; // degrees


// Cals / constants for Light Control
/* KeLC_t_CameraLightDelay: Delay time between enabling the camera light and allowing the data feed to be used. [seconds] */
const double KeLC_t_CameraLightDelay = 0.01;

/* KeLC_t_CameraLightMaxOnTime: Max amount of time to have the camera light enabled. [seconds] */
const double KeLC_t_CameraLightMaxOnTime = 30.0;

/* CeLC_k_BlinkinLED_SolidWhite: Constant for the Blinkin to command solid white. */
const double CeLC_k_BlinkinLED_SolidWhite = 0.93;

/* CeLC_k_BlinkinLED_BreathRed: Constant for the Blinkin to command breath red.  */
const double CeLC_k_BlinkinLED_BreathRed = -0.17;

/* CeLC_k_BlinkinLED_BreathBlue: Constant for the Blinkin to command breath blue.  */
const double CeLC_k_BlinkinLED_BreathBlue = -0.15;

/* CeLC_k_BlinkinLED_LightChaseGray: Constant for the Blinkin to command a light chase gray.  */
const double CeLC_k_BlinkinLED_LightChaseGray = -0.27;

/* CeLC_k_BlinkinLED_RainbowWithGlitter: Constant for the Blinkin to command rainbow with glitter.  */
const double CeLC_k_BlinkinLED_RainbowWithGlitter = -0.89;

/* CeLC_k_BlinkinLED_GreenAndRed: Constant for the Blinkin to command rainbow with glitter.  */
const double CeLC_k_BlinkinLED_GreenAndRed = 0.43;

// Gyro cals
/* KeGRY_ms_GyroTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const int KeGRY_ms_GyroTimeoutMs = 30; // Waits and reports to DS if fails

// Encoder / speed calculation related cals
/* KeENC_k_ReductionRatio: Reduction ratio for swerve drive module. */ /*New chart says 16.6 as reduction*/ 
const double KeENC_k_ReductionRatio = 8.33;

// #ifdef PracticeBot
// const double KeENC_k_ReductionRatio = 8.33;
// #endif

/* KeENC_In_WheelCircumfrence: Circumferance of wheel, in inches (4in nominal diameter). */
const double KeENC_In_WheelCircumfrence = 12.566;

// #ifdef PracticeBot
// const double KeENC_In_WheelCircumfrence = 12.566;
// #endif

/* KeENC_k_SD_VoltageToAngle: Gain that converts the measured voltage of the absolute encoder for the swerve drive angle measurement to an equivalent angle in degrees. (practice bot only) */
static const double KeENC_k_SD_VoltageToAngle = 72.0;

/*****************************************************************************************************
 * Manipulator (MAN) Cals
******************************************************************************************************/
/* KeROBO_t_MotorTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const double KeROBO_t_MotorTimeoutMs = 30;

/* KeMAN_A_ManipulatorNeoCurrentLimHigh: Max allowed current going to each Neo 550 used in the manipulator. [amps] */
const double KeMAN_A_ManipulatorNeoCurrentLimHigh = 40;

/* KeMAN_A_ManipulatorNeoCurrentLim: Max allowed current going to each Neo 550 used in the manipulator. [amps] */
const double KeMAN_A_ManipulatorNeoCurrentLim = 20;

/* KeENC_k_LinearSlideEncoderScaler: Scalar multiplied against the encoder read to translate to degrees relative to inches traveled for the linear slide. */
const double KeENC_k_LinearSlideEncoderScaler = 0.001218;

/* KeENC_k_ArmPivot: Scalar multiplied against the encoder. */
const double KeENC_k_ArmPivot = 2.903;

/* KeENC_RPM_IntakeROllers: Finds the speed of the intake rollers. */
const double KeENC_RPM_IntakeRollers = 1.0;

/* KeENC_RPM_Gripper: Finds the speed of the gripper. */
const double KeENC_RPM_Gripper = 1.0;

/* KeENC_Deg_Wrist: Scalaer to convert encoder reading to actual position of the wrist, how much we've rotated. */
const double KeENC_Deg_Wrist = -1.16883;

/* KaMAN_k_ManipulatorTestPower: Test power output for the manipulator controls. ONLY used in test mode!! */
const double KaMAN_k_ManipulatorTestPower[E_MAN_Sz] = { 0.08, // E_MAN_ArmPivot
                                                        0.25, // E_MAN_LinearSlide
                                                        0.05, // E_MAN_Wrist
                                                       -0.15, // E_MAN_Gripper
                                                       -0.25, // E_MAN_IntakeRollers
                                                        1.0}; // E_MAN_IntakeArm

/* KeMAN_t_StateTimeOUt: Sets transition time out. */
const double KeMAN_t_StateTimeOut = 1.5; // Drop-off

/* KaMAN_k_ArmPivotPID_Gx: PID gains for the Arm Pivot control. */
const double KaMAN_k_ArmPivotPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                             0.000001, // kI
                                                             0.002000, // kD
                                                             0.0,      // kIz
                                                             0.0,      // kFF
                                                             1.0,      // kMaxOut
                                                            -1.0,      // kMinOut
                                                             1.05,     // kMaxVel
                                                             0.5,      // kMinVel
                                                             0.0,      // kMaxAcc
                                                             0.0};     // kAllErr

/* KaMAN_k_LinearSlidePID_Gx: PID gains for the linear slide control. */
const double KaMAN_k_LinearSlidePID_Gx[E_PID_CalSz] = { 0.9,       // P Gx 0.45
                                                        0.0000010, // I Gx
                                                        0.000000,  // D Gx 
                                                        0.99,      // P UL
                                                       -0.99,      // P LL
                                                        0.05,      // I UL
                                                       -0.05,      // I LL
                                                        0.2,       // D UL
                                                       -0.2,       // D LL
                                                        0.99,      // Max upper
                                                       -0.99};     // Max lower

/* KaMAN_k_WristPID_Gx: PID gains for the Wrist control. */
const double KaMAN_k_WristPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                          0.000001, // kI
                                                          0.002000, // kD
                                                          0.0,      // kIz
                                                          0.0,      // kFF
                                                          1.0,      // kMaxOut
                                                         -1.0,      // kMinOut
                                                          1.05,     // kMaxVel
                                                          0.5,      // kMinVel
                                                          0.0,      // kMaxAcc
                                                          0.0};     // kAllErr

/* KaMAN_k_GripperPID_Gx: PID gains for the Gripper control. */
const double KaMAN_k_GripperPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                            0.000001, // kI
                                                            0.002000, // kD
                                                            0.0,      // kIz
                                                            0.0,      // kFF
                                                            1.0,      // kMaxOut
                                                           -1.0,      // kMinOut
                                                            1.05,     // kMaxVel
                                                            0.5,      // kMinVel
                                                            0.0,      // kMaxAcc
                                                            0.0};     // kAllErr

/* KaMAN_k_IntakeRollersPID_Gx: PID gains for the Intake Rollers control. */
const double KaMAN_k_IntakeRollersPID_Gx[E_PID_SparkMaxCalSz] = { 0.00070,  // kP
                                                                  0.000001, // kI
                                                                  0.0,      // kD
                                                                  0.0,      // kIz
                                                                  0.0,      // kFF
                                                                  1.0,      // kMaxOut
                                                                 -1.0,      // kMinOut
                                                                  0.0,      // kMaxVel
                                                                  0.0,      // kMinVel
                                                                 55.0,      // kMaxAcc
                                                                  0.0};     // kAllErr

/* KaMAN_Deg_ArmPivotAngle: sets Arm Pivot final positons for each state */
const double KaMAN_Deg_ArmPivotAngle[E_MAN_State_Sz] = {  0.00,  // Sched - Init
                                                         -0.55,  // Sched - Driving
                                                         39.40,  // Sched - Main Intake
                                                         45.80,  // Sched - Floor Cone Drop
                                                        105.00,  // Sched - Mid Cube Intake
                                                        122.68,  // Sched - Mid Cone Intake
                                                         90.00,  // Sched - High Cube Drop
                                                         40.00,  // Sched - Low Cube Drop   - workaround 54
                                                        118.00,  // Sched - High Cone Drop
                                                        104.00}; // Sched - Low Cone Drop

/* KaMAN_In_LinearSlidePosition: sets LInear Slide final positons for each state */
const double KaMAN_In_LinearSlidePosition[E_MAN_State_Sz] = {  0.0,   // Sched - Init
                                                             -21.65,  // Sched - Driving
                                                               0.968, // Sched - Main Intake
                                                             -21.65,  // Sched - Floor Cone Drop
                                                               5.3,   // Sched - Mid Cube Intake
                                                              -3.66,  // Sched - Mid Cone Intake
                                                              -3.00,  // Sched - High Cube Drop  6.25
                                                             -21.65,  // Sched - Low Cube Drop
                                                               6.14,  // Sched - High Cone Drop
                                                               6.14}; // Sched - Low Cone Drop

/* KaMAN_Deg_WristAngle: sets Wrist final angle for each state */
const double KaMAN_Deg_WristAngle[E_MAN_State_Sz] = {  0.00,  // Sched - Init
                                                      83.35,  // Sched - Driving - 83.35  work around 61
                                                       4.95,  // Sched - Main Intake
                                                      80.00,  // Sched - Floor Cone Drop
                                                     -34.87,  // Sched - Mid Cube Intake
                                                      11.00,  // Sched - Mid Cone Intake
                                                      45.00,  // Sched - High Cube Drop 1.92
                                                      75.00,  // Sched - Low Cube Drop - 75.00  work around 61
                                                      53.90,  // Sched - High Cone Drop
                                                      27.25}; // Sched - Low Cone Drop

/* KeMAN_k_GripperReleaseConeFast: Sets Gripper fast release for cone.  Must be between -1 and 1. */
const double KeMAN_k_GripperReleaseConeFast = -0.35;

/* KeMAN_k_GripperReleaseCubeFast: Sets Gripper fast release for cube.  Must be between -1 and 1. Cube is reverse of cone direction. */
const double KeMAN_k_GripperReleaseCubeFast = 0.8;

/* KeMAN_k_GripperReleaseConeSlow: Sets Gripper slow release for cone.  Must be between -1 and 1. */
const double KeMAN_k_GripperReleaseConeSlow = -0.15;

/* KeMAN_k_GripperReleaseCubeSlow: Sets Gripper slow release for cube.  Must be between -1 and 1. Cube is reverse of cone direction. */
const double KeMAN_k_GripperReleaseCubeSlow = 0.15;

/* KeMAN_k_GripperIntakeCone: Sets Gripper cone intake power */
const double KeMAN_k_GripperIntakeCone = 0.4;  // 0.4

/* KeMAN_k_GripperIntakeCube: Sets Gripper cube intake power */
const double KeMAN_k_GripperIntakeCube = -0.35;  //-0.45

/* KeMAN_k_GripperIntakeholdCone: Sets Gripper intake hold */
const double KeMAN_k_GripperIntakeholdCone = 0.05; 

/* KeMAN_k_GripperIntakeholdCube: Sets Gripper intake hold */
const double KeMAN_k_GripperIntakeholdCube = -0.10;

/* KeMAN_t_GripperOnTm: Amount of time gripper will remain on after it is initially commanded on. */
const double KeMAN_t_GripperOnTm = 0.5;

/* KeMAN_t_GripperPullInTm: Amount of time gripper will continue pulling in after object is detected. */
const double KeMAN_t_GripperPullInTm = 0.15;

/* KaMAN_RPM_IntakePower: sets Intake power for each state */
const double KaMAN_RPM_IntakePower[E_MAN_State_Sz] = {  0.0,   // Sched - Init
                                                        0.0,   // Sched - Driving
                                                       -0.45,  // Sched - Main Intake
                                                        0.0,   // Sched - Floor Cone Drop
                                                        0.0,   // Sched - Mid Cube Intake
                                                        0.0,   // Sched - Mid Cone Intake
                                                        0.0,   // Sched - High Cube Drop
                                                        0.0,   // Sched - Low Cube Drop
                                                        0.0,   // Sched - High Cone Drop
                                                        0.0};  // Sched - Low Cone Drop

/* KaMAN_e_IntakePneumatics: sets the Pneumatics either true (arm extended) or false (arm retracted) for each state */
const T_MotorControlType KaMAN_e_IntakePneumatics[E_MAN_State_Sz] = {E_MotorRetract,  // Sched - Init
                                                                     E_MotorRetract,  // Sched - Driving
                                                                     E_MotorExtend,   // Sched - Main Intake
                                                                     E_MotorRetract,  // Sched - Floor Cone Drop
                                                                     E_MotorRetract,  // Sched - Mid Cube Intake
                                                                     E_MotorRetract,  // Sched - Mid Cone Intake
                                                                     E_MotorRetract,  // Sched - High Cube Drop
                                                                     E_MotorRetract,  // Sched - Low Cube Drop
                                                                     E_MotorRetract,  // Sched - High Cone Drop
                                                                     E_MotorRetract}; // Sched - Low Cone Drop

/* KaMAN_DegS_ArmPivotRate: Table that contains the pivot rate of the arm. */
const double KaMAN_DegS_ArmPivotRate[E_MAN_State_Sz][E_MAN_State_Sz] =  // [Cmnd][Attnd]
  {
    {0.15,  0.15,    0.15,        0.15,              0.15,            0.15,            0.15,           0.15,          0.15,           0.15}, // Cmnd - Init
    {0.15,  0.28,    0.15,        0.15,              0.15,            0.15,            0.15,           0.15,          0.15,           0.15}, // Cmnd - Driving
    {0.15,  0.15,    0.15,        0.15,              0.15,            0.15,            0.15,           0.15,          0.15,           0.15}, // Cmnd - Main Intake
    {0.15,  0.28,    0.15,        0.15,              0.15,            0.15,            0.15,           0.15,          0.15,           0.15}, // Cmnd - Floor Cone Drop
    {0.15,  0.28,    0.15,        0.15,              0.28,            0.15,            0.28,           0.28,          0.15,           0.28}, // Cmnd - Mid Cube Intake
    {0.15,  0.28,    0.15,        0.15,              0.28,            0.28,            0.28,           0.28,          0.28,           0.28}, // Cmnd - Mid Cone Intake
    {0.15,  0.28,    0.15,        0.15,              0.28,            0.15,            0.28,           0.28,          0.15,           0.28}, // Cmnd - High Cube Drop
    {0.15,  0.28,    0.15,        0.15,              0.15,            0.15,            0.15,           0.28,          0.15,           0.15}, // Cmnd - Low Cube Drop
    {0.15,  0.28,    0.15,        0.15,              0.28,            0.15,            0.28,           0.28,          0.28,           0.28}, // Cmnd - High Cone Drop
    {0.15,  0.28,    0.15,        0.15,              0.15,            0.15,            0.15,           0.28,          0.15,           0.28}  // Cmnd - Low Cone Drop
  // Init:  Driving: Main Intake: Floor Cone Drop:   Mid Cube Intake: Mid Cone Intake: High Cube Drop: Low Cube Drop: High Cone Drop: Low Cone Drop -> Attnd
  };

/* KaMAN_InS_LinearSlideRate: Table that contains the linear slide transition rate. */
const double KaMAN_InS_LinearSlideRate[E_MAN_State_Sz][E_MAN_State_Sz] =  // [Cmnd][Attnd]
  {
    {9.00,  0.12,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - Init
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - Driving
    {0.22,  1.25,    0.22,        0.22,              0.22,            0.22,            0.22,           0.22,          0.22,           0.22}, // Cmnd - Main Intake
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - Floor Cone Drop
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - Mid Cube Intake
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - Mid Cone Intake
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - High Cube Drop
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - Low Cube Drop
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}, // Cmnd - High Cone Drop
    {9.00,  9.00,    9.00,        9.00,              9.00,            9.00,            9.00,           9.00,          9.00,           9.00}  // Cmnd - Low Cone Drop
  // Init:  Driving: Main Intake: Floor Cone Drop:   Mid Cube Intake: Mid Cone Intake: High Cube Drop: Low Cube Drop: High Cone Drop: Low Cone Drop -> Attnd
  };

/* KaMAN_DegS_WristRate: Table that contains the wrist transition rate. */
const double KaMAN_DegS_WristRate[E_MAN_State_Sz][E_MAN_State_Sz] =  // [Cmnd][Attnd]
  {
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Init
    {0.65,  0.45,    0.65,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Driving
    {0.45,  0.75,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Main Intake
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Floor Cone Drop
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Mid Cube Intake
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Mid Cone Intake
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - High Cube Drop
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - Low Cube Drop
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}, // Cmnd - High Cone Drop
    {0.45,  0.45,    0.45,        0.45,              0.45,            0.45,            0.45,           0.45,          0.45,           0.45}  // Cmnd - Low Cone Drop
  // Init:  Driving: Main Intake: Floor Cone Drop:   Mid Cube Intake: Mid Cone Intake: High Cube Drop: Low Cube Drop: High Cone Drop: Low Cone Drop -> Attnd
  };

/* KaMAN_Deg_ArmPivotDb: Sets Arm Pivot dead band */
const double KaMAN_Deg_ArmPivotDb[E_MAN_State_Sz] = {4.0,  // Sched - Init
                                                     4.0,  // Sched - Driving
                                                     4.0,  // Sched - Main Intake
                                                     4.0,  // Sched - Floor Cone Drop
                                                     4.0,  // Sched - Mid Cube Intake
                                                     4.0,  // Sched - Mid Cone Intake
                                                     4.0,  // Sched - High Cube Drop
                                                     4.0,  // Sched - Low Cube Drop
                                                     4.0,  // Sched - High Cone Drop
                                                     4.0}; // Sched - Low Cone Drop

/* KaMAN_In_LinearSlideDb: Sets LInear Slide dead band. */
const double KaMAN_In_LinearSlideDb[E_MAN_State_Sz] = {0.5,  // Sched - Init
                                                       0.5,  // Sched - Driving
                                                       0.5,  // Sched - Main Intake
                                                       0.5,  // Sched - Floor Cone Drop
                                                       0.5,  // Sched - Mid Cube Intake
                                                       0.5,  // Sched - Mid Cone Intake
                                                       0.5,  // Sched - High Cube Drop
                                                       0.5,  // Sched - Low Cube Drop
                                                       0.5,  // Sched - High Cone Drop
                                                       0.5}; // Sched - Low Cone Drop

/* KaMAN_Deg_WristDb: sets Wrist final angle for each state */
const double KaMAN_Deg_WristDb[E_MAN_State_Sz] = {1.0,  // Sched - Init
                                                  1.0,  // Sched - Driving
                                                  1.0,  // Sched - Main Intake
                                                  1.0,  // Sched - Floor Cone Drop
                                                  1.0,  // Sched - Mid Cube Intake
                                                  1.0,  // Sched - Mid Cone Intake
                                                  1.0,  // Sched - High Cube Drop
                                                  1.0,  // Sched - Low Cube Drop
                                                  1.0,  // Sched - High Cone Drop
                                                  1.0}; // Sched - Low Cone Drop

/* KaMAN_e_ControllingTable: Table that contains the commanded state of the manipulator and intake based on the current attained state and schedueld state. */
const TeMAN_ManipulatorStates KaMAN_e_ControllingTable[E_MAN_State_Sz][E_MAN_State_Sz] =  // [Sched][Attnd]
  {
    {E_MAN_Init,    E_MAN_Init,            E_MAN_MainIntake,  E_MAN_FloorConeDrop,    E_MAN_MidCubeIntake,   E_MAN_MidConeIntake,   E_MAN_HighCubeDrop,    E_MAN_LowCubeDrop,     E_MAN_HighConeDrop,    E_MAN_LowConeDrop},     // Sched - Init
    {E_MAN_Driving, E_MAN_Driving,         E_MAN_Driving,     E_MAN_Driving,          E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving},         // Sched - Driving
    {E_MAN_Driving, E_MAN_MainIntake,      E_MAN_MainIntake,  E_MAN_Driving,          E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving,         E_MAN_Driving},         // Sched - Main Intake
    {E_MAN_Driving, E_MAN_FloorConeDrop,   E_MAN_Driving,     E_MAN_FloorConeDrop,    E_MAN_FloorConeDrop,   E_MAN_FloorConeDrop,   E_MAN_FloorConeDrop,   E_MAN_FloorConeDrop,   E_MAN_FloorConeDrop,   E_MAN_FloorConeDrop},   // Sched - Floor Cone Intake
    {E_MAN_Driving, E_MAN_MidCubeIntake,   E_MAN_Driving,     E_MAN_MidCubeIntake,    E_MAN_MidCubeIntake,   E_MAN_MidCubeIntake,   E_MAN_MidCubeIntake,   E_MAN_MidCubeIntake,   E_MAN_MidCubeIntake,   E_MAN_MidCubeIntake},   // Sched - Mid Cube Intake
    {E_MAN_Driving, E_MAN_MidConeIntake,   E_MAN_Driving,     E_MAN_MidConeIntake,    E_MAN_MidConeIntake,   E_MAN_MidConeIntake,   E_MAN_MidConeIntake,   E_MAN_Driving,         E_MAN_MidConeIntake,   E_MAN_MidConeIntake},   // Sched - Mid Cone Intake
    {E_MAN_Driving, E_MAN_HighCubeDrop,    E_MAN_Driving,     E_MAN_HighCubeDrop,     E_MAN_HighCubeDrop,    E_MAN_HighCubeDrop,    E_MAN_HighCubeDrop,    E_MAN_HighCubeDrop,    E_MAN_HighCubeDrop,    E_MAN_HighCubeDrop},    // Sched - High Cube Drop
    {E_MAN_Driving, E_MAN_LowCubeDrop,     E_MAN_Driving,     E_MAN_LowCubeDrop,      E_MAN_LowCubeDrop,     E_MAN_LowCubeDrop,     E_MAN_LowCubeDrop,     E_MAN_LowCubeDrop,     E_MAN_LowCubeDrop,     E_MAN_LowCubeDrop},     // Sched - Low Cube Drop
    {E_MAN_Driving, E_MAN_HighConeDrop,    E_MAN_Driving,     E_MAN_HighConeDrop,     E_MAN_HighConeDrop,    E_MAN_HighConeDrop,    E_MAN_HighConeDrop,    E_MAN_HighConeDrop,    E_MAN_HighConeDrop,    E_MAN_HighConeDrop},    // Sched - High Cone Drop
    {E_MAN_Driving, E_MAN_LowConeDrop,     E_MAN_Driving,     E_MAN_LowConeDrop,      E_MAN_LowConeDrop,     E_MAN_LowConeDrop,     E_MAN_LowConeDrop,     E_MAN_LowConeDrop,     E_MAN_LowConeDrop,     E_MAN_LowConeDrop}      // Sched - Low Cone Drop
  };


// Constants and cals for Swerve Drive (SD) control:
/* C_SD_L: Robot wheelbase. [meters] */
const double C_SD_L = 0.5969;

/* C_SD_W: Robot track width. [meters] */
const double C_SD_W = 0.5969;

/* C_SD_R: Constant composed of the C_SD_W and C_SD_L constants: R = sqrt(L^2 + W^2) [meters]*/
const double C_SD_R = 0.8441;

/* K_SD_SteerMotorCurrentLimit: Max allowed current going to each swerve drive steer motor. */
const double K_SD_SteerMotorCurrentLimit = 25;

/* KeENC_Deg_SD_WheelOffsetAngle: Offset angle for each respective corder of the swerve drive wheel.  This is the angle
   reading from the absolute encoder that is indicated in order for the wheel to point straight. */
const double KeENC_Deg_SD_WheelOffsetAngle[E_RobotCornerSz] = {158.291015625,  // E_FrontLeft 152.578125
                                                               218.14453125,  // E_FrontRight 212.783203
                                                               117.509765625,  // E_RearLeft 118.740234
                                                                82.79296875}; // E_RearRight 76.289063

/* KeENC_k_WheelOffsetAnglePractieBot: Offset angle for each respective corder of the swerve drive wheel.  This is the angle
   reading from the absolute encoder that is indicated in order for the wheel to point straight.  For practice bot only. */
const double KeENC_k_WheelOffsetAnglePractieBot[E_RobotCornerSz] = {-179.4,  // E_FrontLeft 1.3  -176
                                                                     -16.1,  // E_FrontRight 163.5
                                                                      52.9,  // E_RearLeft 230.8  -127.1
                                                                      96.9}; // E_RearRight 282.0

/* K_SD_WheelGx: Gain multiplied by each calculated desired speed.  Intended to account for variation in wheel size. */
const double K_SD_WheelGx[E_RobotCornerSz] = {-1.0,  // E_FrontLeft
                                              -1.0,  // E_FrontRight
                                              -1.0,  // E_RearLeft
                                              -1.0}; // E_RearRight

/* KeDRC_k_SD_MinGain: Min gain applied to the wheel speed for swerve drive. */
const double KeDRC_k_SD_MinGain = 0.2;

/* KeDRC_k_SD_AutonGain: Gain applied to the auton commands coming in for swerve drive. */
const double KeDRC_k_SD_AutonGain = 0.9;

/* K_SD_MaxGain: Max gain allowed for swerve drive control. */
const double K_SD_MaxGain = 0.7;

/* KeDRC_k_SD_ArmExtendedGx: Gain with the arm extended. */
const double KeDRC_k_SD_ArmExtendedGx = 0.3;

/* Ke_SD_AutoCorrectPID_Gx: PID gains for the auto correct.  PID control is within the RoboRio.  */
const double Ke_SD_AutoCorrectPID_Gx[E_PID_CalSz] = {  40.0,    // P Gx  75
                                                        0.5,    // I Gx 0.03
                                                        0.0005, // D Gx 0.0005
                                                      300.0,    // P UL 0.6
                                                     -300.0,    // P LL -0.4
                                                      300.0,    // I UL 0.12
                                                     -300.0,    // I LL -0.12
                                                        1.0,    // D UL 0.5
                                                       -1.0,    // D LL -0.5
                                                      400.0,    // Max upper 0.9
                                                     -400.0};   // Max lower -0.9

/* Ke_k_SD_AutoCorrectMaxWheelOffset: Max percent offset of wheelspeed.*/
const double Ke_k_SD_AutoCorrectMaxWheelOffset = 0.8;

/* K_SD_WheelMaxSpeed: Max RPM speed of the swerve drive wheel motor.*/
const double K_SD_WheelMaxSpeed = 6000;

/* Ke_RPM_SD_WheelMinCmndSpeed: Min RPM speed of the swerve drive wheel to keep it under PID control.
  If the absolute value of the command, wheels will transition to 0 power (but still in brake
  mode).  There is a corresponding actual speed threshold. [RPM] */
const double Ke_RPM_SD_WheelMinCmndSpeed = 0.2;

/* K_SD_WheelSpeedPID_V2_Gx: PID gains for the driven wheels that is within the motor controllers. */
const double K_SD_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz] = { 0.000350, // kP
                                                               0.000001, // kI
                                                               0.000001, // kD
                                                               0.0,      // kIz
                                                               0.0,      // kFF
                                                               1.0,      // kMaxOutput
                                                              -1.0,      // kMinOutput
                                                               0.0,      // kMaxVel
                                                               0.0,      // kMinVel
                                                              150.0,     // kMaxAcc 150
                                                               0.0};     // kAllErr

/* K_SD_WheelAnglePID_Gx: PID gains for the angle of the swerve drive wheels.  PID control is within the RoboRio.  */
const double K_SD_WheelAnglePID_Gx[E_PID_CalSz] = { 0.0035,   // P Gx  0.002
                                                    0.000001, // I Gx 0.000001
                                                    0.000005, // D Gx 0.0000005
                                                    1.0,      // P UL 0.6
                                                   -1.0,      // P LL -0.4
                                                    0.15,     // I UL 0.12
                                                   -0.15,     // I LL -0.12
                                                    1.0,      // D UL 0.5
                                                   -1.0,      // D LL -0.5
                                                    1.0,      // Max upper 0.9
                                                   -1.0};     // Max lower -0.9

/* K_SD_WheelAnglePID_GxPracticeBot: PID gains for the angle of the swerve drive wheels on practice bot.  PID control is within the RoboRio.  */
const double K_SD_WheelAnglePID_GxPracticeBot[E_PID_CalSz] = { 0.009,      // P Gx  0.002
                                                               0.000001,  // I Gx 0.000001
                                                               0.0000005, // D Gx 0.0000005
                                                               0.9,       // P UL 0.6
                                                              -0.9,       // P LL -0.4
                                                               0.15,      // I UL 0.12
                                                              -0.15,      // I LL -0.12
                                                               0.7,       // D UL 0.5
                                                              -0.7,       // D LL -0.5
                                                               0.9,       // Max upper 0.9
                                                              -0.9};      // Max lower -0.9

/* K_SD_DesiredDriveSpeedAxis: Joystick scale axis for K_SD_DesiredDriveSpeed.  */
const double K_SD_DesiredDriveSpeedAxis[20] = {-0.95,
                                               -0.85,
                                               -0.75,
                                               -0.65,
                                               -0.55,
                                               -0.45,
                                               -0.35,
                                               -0.25,
                                               -0.15,
                                               -0.05,
                                                0.05,
                                                0.15,
                                                0.25,
                                                0.35,
                                                0.45,
                                                0.55,
                                                0.65,
                                                0.75,
                                                0.85,
                                                0.95};

/* K_SD_DesiredDriveSpeed: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double K_SD_DesiredDriveSpeed[20] = {-1.00,  //-1
                                           -0.88,  //-0.88
                                           -0.50,  //-0.6
                                           -0.35,  //-0.4
                                           -0.20,  //-0.25
                                           -0.15,  //-0.2
                                           -0.10,  //-0.15
                                           -0.06,  //-0.1
                                           -0.02,  //-0.05
                                            0.00,  //-0.0
                                            0.00,  // 0.0
                                            0.02,  // 0.15
                                            0.06,  // 0.25
                                            0.10,  // 0.35
                                            0.15,  // 0.45
                                            0.20,  // 0.55
                                            0.35,  // 0.65
                                            0.50,  // 0.75
                                            0.88,  // 0.85
                                            1.00}; // 0.95

/* K_SD_DesiredRotateAxis: Joystick scale axis for K_SD_DesiredDriveSpeed.  */
const double K_SD_DesiredRotateAxis[20] = {-0.95,
                                           -0.85,
                                           -0.75,
                                           -0.65,
                                           -0.55,
                                           -0.45,
                                           -0.35,
                                           -0.25,
                                           -0.15,
                                           -0.05,
                                            0.05,
                                            0.15,
                                            0.25,
                                            0.35,
                                            0.45,
                                            0.55,
                                            0.65,
                                            0.75,
                                            0.85,
                                            0.95};

/* K_SD_DesiredRotate: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double K_SD_DesiredRotate[20] = {-1.00,  //-1
                                       -0.50,  //-0.88
                                       -0.20,  //-0.6
                                       -0.11,  //-0.4
                                       -0.09,  //-0.25
                                       -0.07,  //-0.2
                                       -0.05,  //-0.15
                                       -0.03,  //-0.1
                                       -0.01,  //-0.05
                                        0.00,  //-0.0
                                        0.00,  // 0.0
                                        0.01,  // 0.15
                                        0.03,  // 0.25
                                        0.05,  // 0.35
                                        0.07,  // 0.45
                                        0.09,  // 0.55
                                        0.11,  // 0.65
                                        0.20,  // 0.75
                                        0.50,  // 0.85
                                        1.00}; // 0.95

/* KnLU_k_SD_DesiredAccelAxis: Joystick scale axis for KtLU_k_SD_DesiredAccel.  */
const double KnLU_k_SD_DesiredAccelAxis[20] = {-0.95,
                                               -0.85,
                                               -0.75,
                                               -0.65,
                                               -0.55,
                                               -0.45,
                                               -0.35,
                                               -0.25,
                                               -0.15,
                                               -0.10,
                                                0.10,
                                                0.15,
                                                0.25,
                                                0.35,
                                                0.45,
                                                0.55,
                                                0.65,
                                                0.75,
                                                0.85,
                                                0.95};

/* KtLU_k_SD_DesiredAccel: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double KtLU_k_SD_DesiredAccel[20] = {-1.00,  //-1
                                           -0.88,  //-0.88
                                           -0.50,  //-0.6
                                           -0.35,  //-0.4
                                           -0.20,  //-0.25
                                           -0.15,  //-0.2
                                           -0.10,  //-0.15
                                           -0.06,  //-0.1
                                           -0.02,  //-0.05
                                            0.00,  //-0.0
                                            0.00,  // 0.0
                                            0.02,  // 0.15
                                            0.06,  // 0.25
                                            0.10,  // 0.35
                                            0.15,  // 0.45
                                            0.20,  // 0.55
                                            0.35,  // 0.65
                                            0.50,  // 0.75
                                            0.88,  // 0.85
                                            1.00}; // 0.95

/* RotateDeadBand: Check Rotation value approx 0 */
const double K_SD_RotateDeadBand = 0.05;

/* KeDRC_Pct_SD_StrafeDB: Check Rotation value approx 0 */
const double KeDRC_Pct_SD_StrafeDB = 0.75;

/* Ke_k_SD_SignX: Determines sign of the calculation for the X component of the swerve drive offset. */
const double Ke_k_SD_SignX[E_RobotCornerSz] = { 1.0,  // E_FrontLeft
                                                1.0,  // E_FrontRight 
                                               -1.0,  // E_RearLeft
                                               -1.0}; // E_RearRight 

/* Ke_k_SD_SignY: Determines sign of the calculation for the Y component of the swerve drive offset. */
const double Ke_k_SD_SignY[E_RobotCornerSz] = { 1.0,  // E_FrontLeft
                                               -1.0,  // E_FrontRight 
                                                1.0,  // E_RearLeft
                                               -1.0}; // E_RearRight 

/* Ke_k_SD_CorrectionGx: Correction gain for swereve drive auto center. */
const double Ke_k_SD_CorrectionGx = 1;

/* ADAS Cals */
/* K_ADAS_DM_DriveTimeLong: This is the default drive forward time. [Seconds] */
const double K_ADAS_DM_DriveTimeLong = 5.5;

/* K_ADAS_DM_DriveFWD_Pct: This is the default drive forward Pct. [Pct] */
const double K_ADAS_DM_DriveFWD_Pct = -0.3;

/* KeADAS_t_DM_DriveTimeFar: This is the drive far time. [Seconds] */
const double KeADAS_t_DM_DriveTimeFar = 11.5;

/* KeADAS_Pct_DM_DriveFWD_Far: This is the default drive forward Pct. [Pct] */
const double KeADAS_Pct_DM_DriveFWD_Far = -0.4;

/* KeADAS_t_DM_RevDriveTime: Time to drive in reverse. [Seconds] */
const double KeADAS_t_DM_RevDriveTime = 1.0;

/* KeADAS_Pct_DM_RevDrive: This is the reverse drive  Pct. [Pct] */
const double KeADAS_Pct_DM_RevDrive = 0.4;

/* K_ADAS_DM_RotateDebounceTime: This is the debounce time for the DM rotate state. [seconds] */
const double K_ADAS_DM_RotateDebounceTime = 0.02;
 
/* KeADAS_t_DM_PathFollowDebounceTime: This is the debounce time for the DM pathfollow state. [seconds] */
const double KeADAS_t_DM_PathFollowDebounceTime = 0.045;

/* K_ADAS_DM_RotateDeadbandAngle: This is the deband angle for the DM rotate state. [degrees] */
const double K_ADAS_DM_RotateDeadbandAngle = 2.0;

/* K_ADAS_DM_XY_Deadband: This is the deband position for the DM XY drive state. [meters] */
const double K_ADAS_DM_XY_Deadband = 3.0;

/* KeADAS_Deg_DM_AutoBalanceDb: This is the deband angle for the DM auto balance state. [degrees] */
const double KeADAS_Deg_DM_AutoBalanceDb = 10.0;

/* KeADAS_t_DM_AutoBalanceDb: This is the debounce time for the DM auto balance state. [seconds] */
const double KeADAS_t_DM_AutoBalanceDb = 0.06;

/* KeADAS_t_DM_AutoBalanceHold: This is the amount of time for the DM auto balance to hold. [seconds] */
const double KeADAS_t_DM_AutoBalanceHold = 10;

/* KeADAS_k_DM_AutoBalanceFastPID: This is the PID gains for the auto balance. */
const double KeADAS_k_DM_AutoBalanceFastPID[E_PID_CalSz] = { 0.037,     // P Gx
                                                             0.0000007, // I Gx
                                                             0.00001,   // D Gx
                                                             0.8,       // P UL
                                                            -0.8,       // P LL
                                                             0.03,      // I UL
                                                            -0.03,      // I LL
                                                             0.5,       // D UL
                                                            -0.5,       // D LL
                                                             0.6,       // Max upper
                                                            -0.6};      // Max lower

/* KeADAS_k_DM_AutoBalanceSlowPID: This is the PID gains for the auto balance. */
const double KeADAS_k_DM_AutoBalanceSlowPID[E_PID_CalSz] = { 0.037,     // P Gx
                                                             0.0000007, // I Gx
                                                             0.00001,   // D Gx
                                                             0.8,       // P UL
                                                            -0.8,       // P LL
                                                             0.03,      // I UL
                                                            -0.03,      // I LL
                                                             0.5,       // D UL
                                                            -0.5,       // D LL
                                                             0.60,      // Max upper
                                                            -0.60};     // Max lower

/* KeADAS_t_DM_TagCenteringDb: This is the deband position for the DM drive state auto centering. [sec] */
const double KeADAS_t_DM_TagCenteringDb = 0.1;

/* KeADAS_Deg_DM_AutoMountDetect: Amount of angle to indicate when bot has mounted onto the charge station. [degrees] */
const double KeADAS_Deg_DM_AutoMountDetect = 3;

/* KeADAS_t_DM_AutoMountDb: Debounce time for auto mount. [sec] */
const double KeADAS_t_DM_AutoMountDb = 0.2;

/* KeADAS_t_DM_AutoMountOnlyDb: Debounce time for auto mount only, not used for mount/dismount. [sec] */
const double KeADAS_t_DM_AutoMountOnlyDb = 1.0;

/* KeADAS_t_DM_AutoMountRevDb: Debounce time for auto mount. [sec] */
const double KeADAS_t_DM_AutoMountRevDb = 0.85;

/* KeADAS_Pct_DM_AutoMountPwr: Power command when in auto mount. [pct] */
const double KeADAS_Pct_DM_AutoMountPwr = -0.50;

/* KeADAS_Pct_DM_AutoMountPwrSlow: Power command when in auto mount. [pct] */
const double KeADAS_Pct_DM_AutoMountPwrSlow = -0.38;

/* KeADAS_t_DM_StopTm: Amount of time to have the robot stopped. [sec] */
const double KeADAS_t_DM_StopTm = 0.1;

/*  Auto Rotation calibrations */
/* K_DesiredRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredRotateSpeedAxis[10] = {-20.0,
                                              -4.0,
                                              -2.0,
                                              -1.0,
                                              -0.2,
                                               0.2,
                                               1.0,
                                               2.0,
                                               4.0,
                                              20.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredRotateSpeed[10] = {-0.60,  // -20.0
                                         -0.12,  //  -4.0
                                         -0.035, //  -2.0
                                         -0.018, //  -1.0
                                          0.02,  //  -0.2
                                          0.02,  //   0.2
                                          0.018, //   1.0
                                          0.035, //   2.0
                                          0.012, //   4.0
                                          0.60}; //  20.0

/* K_DesiredAutoRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredAutoRotateSpeedAxis[10] = {-4.0,
                                                 -3.0,
                                                 -2.0,
                                                 -1.0,
                                                 -0.2,
                                                  0.2,
                                                  1.0,
                                                  2.0,
                                                  3.0,
                                                  4.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredAutoRotateSpeed[10] = {-0.15,  //  -4.0
                                             -0.02,  //  -3.0
                                             -0.008, //  -2.0
                                             -0.005, //  -1.0
                                              0.00,  //  -0.2
                                              0.00,  //   0.2
                                             -0.005, //   1.0
                                              0.008, //   2.0
                                              0.02,  //   3.0
                                              0.15}; //   4.0

/************************************************************************************************************************
 *
 *
 *
 * Depricated cals below, needs to be cleaned up
 *
 *
 *
 *************************************************************************************************************************/

// This is the desired target angle for the auto vision targeting.  This is due to the offset of the camera. For 2020 - 3.3
const double K_TargetVisionAngleUpper = 0.0;

const double K_TargetVisionAngleMin = 10;

const double K_TargetVisionAngleMax = 50;

const double K_TargetVisionDistanceMin = 50;

const double K_TargetVisionDistanceMax = 50;

const double K_TargetVisionAngleErrorMax = 2;

const double K_TargetVisionUpperRollerErrorMax = 200;

const double K_TargetVisionLowerRollerErrorMax = 200;

#define K_BallLauncherDistanceSz 5
#define K_BallLauncherAngleSz 3

const double K_BallLauncherDistanceAxis[K_BallLauncherDistanceSz] = {400, 700, 968, 1300, 1660};

const double K_BallLauncherAngleAxis[K_BallLauncherAngleSz] = {-45, 0, 45};

const double K_BallLauncherRobotAngle[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45},
    {45, 0, -45}
  };

const double K_BallLauncherUpperSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {-1800, -1800, -1800},
    {-1800, -1800, -1800},
    {-1800, -1800, -1800},
    {-2000, -2000, -2000},
    {-2850, -2850, -2850}
  };

const double K_BallLauncherLowerSpeed[K_BallLauncherDistanceSz][K_BallLauncherAngleSz] =
  {
    {-2100, -2100, -2100},
    {-2100, -2100, -2100},
    {-2100, -2100, -2100},
    {-2500, -2500, -2500},
    {-2900, -2900, -2900}
  };

/* Auton specific cals */
const double KaADAS_k_AutonXY_PID_Gx[E_PID_CalSz] = { 0.050,       // P Gx
                                                      0.00060,    // I Gx 0.000001
                                                      0.00007,      // D Gx
                                                      0.60,       // P UL
                                                     -0.60,       // P LL
                                                      0.3,      // I UL
                                                     -0.3,      // I LL
                                                      0.5,       // D UL
                                                     -0.5,       // D LL
                                                      1.0,       // Max upper
                                                     -1.0};      // Max lower

const double KaADAS_k_AutonRotatePID_Gx[E_PID_CalSz] = { 0.00020,     // P Gx
                                                         0.00020, // I Gx
                                                         0.00007,  // D Gx
                                                         0.29,      // P UL
                                                        -0.29,      // P LL
                                                         0.175,     // I UL
                                                        -0.175,     // I LL
                                                         0.16,      // D UL
                                                        -0.16,      // D LL
                                                         0.4,      // Max upper
                                                        -0.4};     // Max lower


