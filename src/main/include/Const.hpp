#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired test state here: COMP (no test), BallHandlerTest, Manipulator_Test, DriveMotorTest, WheelAngleTest, ADAS_UT_Test, ADAS_BT_Test
#define COMP
// Define the bot type: CompBot, PracticeBot
#define CompBot

#define NewVision // NewVision or OldVision

#ifdef PracticeBot
const double C_VisOffsetY = 1.0;
const double C_VisOffsetX = 2.0;

#endif

#ifdef CompBot
const double C_VisOffsetY = 3.125;
const double C_VisOffsetX = 13.5;
#endif


// RoboRio controller execution time
const double C_ExeTime = 0.02; // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms

// Amount of time for end game
const double C_End_game_time = 30;

// Numerical constants
const double C_RadtoDeg = 57.2957795;
const double C_Deg2Rad = 0.017453292519943295;
const double C_MeterToIn = 39.37008;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;

static const double KeENC_k_EncoderToAngle = 1; // Raw output of PWM encoder to degrees




 const double C_Tag1Y = 1.071626 * C_MeterToIn; // all these come in meters, we need them in inches to match odometry
  // coord of tag ID 2 and 7
 const double C_Tag2Y = 2.748026 * C_MeterToIn;
  // coord of tag ID 3 and 6
 const double C_Tag3Y = 4.424426 * C_MeterToIn;
 const double C_TagXred = 15.513558 * C_MeterToIn;
const  double C_TagXblue = 1.02743 * C_MeterToIn;


const double C_TagAlignBasePower = 0.02;
const double C_TagScoreOffsetXCube = 34.0; //that little space our bumper is against to score cubes
const double C_TagScoreOffsetYCube = 0.0;  

const double C_TagScoreOffsetXCone = 35.5; //that little space our bumper is against to score cubes
const double C_TagScoreOffsetYCone = 29.0 ;  

// CAN Device IDs:
static const int C_PDP_ID = 21;
static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
static const int KeMAN_i_TurretRotate = 9; 
static const int KeMAN_i_LinearSlide = 10;
static const int KeMAN_i_ArmPivot = 11;
static const int KeMAN_i_Wrist = 12;
static const int KeMAN_i_Gripper = 13;
static const int KeINT_i_IntakeRollers = 14;
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
static const int C_XY_LimitSwitch_ID = 4, C_XD_LimitSwitch_ID = 6, C_IR_Sensor_ID = 3, C_CameraLightControl_ID = 7;
static const int C_LowerBallSensorID = 5;
static const int C_TurretSensorID = 10;


// PWM IDs:
static const int C_VanityLight_ID = 0;


// Vision Cals:
const double K_MoveToTagMovementDeadbandX = 0.5; // inches
const double K_MoveToTagMovementDeadbandY = 0.05; // inches

// const double K_MoveToTagRotationDeadband = 2.0; // degrees

// cals for top target cam

/* K_VisionCalculationDelayTime: Delay time before allowing calculations to occur */
const double K_VisionCalculationDelayTime = 0.1;

/* K_VisionYawLagFilter: First order lag filter coefficents for yaw calculation. */
const double K_VisionYawLagFilter[E_CamLocSz] = {0.05,  // -> top
                                                 0.08}; // -> bottom

/* K_VisionTargetDistLagFilter: First order lag filter coefficents for distance calculation. */
const double K_VisionTargetDistLagFilter[E_CamLocSz] = {0.5,  // -> top
                                                        0.5}; // -> bottom



// Cals / constants for Light Control
/* K_CameraLightDelay: Delay time between enabling the camera light and allowing the data feed to be used. [seconds] */
const double K_CameraLightDelay = 0.01;

/* K_CameraLightMaxOnTime: Max amount of time to have the camera light enabled. [seconds] */
const double K_CameraLightMaxOnTime = 30.0;

/* C_BlinkinLED_SolidWhite: Constant for the Blinkin to command solid white. */
const double C_BlinkinLED_SolidWhite = 0.93;

/* C_BlinkinLED_BreathRed: Constant for the Blinkin to command breath red.  */
const double C_BlinkinLED_BreathRed = -0.17;

/* C_BlinkinLED_BreathBlue: Constant for the Blinkin to command breath blue.  */
const double C_BlinkinLED_BreathBlue = -0.15;

/* C_BlinkinLED_LightChaseGray: Constant for the Blinkin to command a light chase gray.  */
const double C_BlinkinLED_LightChaseGray = -0.27;

/* C_BlinkinLED_RainbowWithGlitter: Constant for the Blinkin to command rainbow with glitter.  */
const double C_BlinkinLED_RainbowWithGlitter = -0.89;


// Gyro cals
/* KeGRY_ms_GyroTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const int KeGRY_ms_GyroTimeoutMs = 30; //Waits and reports to DS if fails

// Encoder / speed calculation related cals
const double KeENC_k_ReductionRatio = 8.33; //Reduction ratio for swerve drive module
const double KeENC_In_WheelCircumfrence = 12.566; // Circumferance of wheel, in inches (4in nominal diameter)
const double KeEnc_Deg_InvalDelta = 7;

/* KeENC_k_SD_VoltageToAngle: Gain that converts the measured voltage of the absolute encoder for the swerve drive angle measurement to an equivalent angle in degrees. (practice bot only) */
static const double KeENC_k_SD_VoltageToAngle = 72.0;

// Turret cals
/* KeENC_k_TurretVoltageToAng: Scalar multiplied against the encoder voltage output to translate to degrees. */
const double KeENC_k_TurretVoltageToAng = 75.28; //72

/* KeENC_Deg_TurretOffset: Offset of the absolute encoder used on the turret. */
// const double KeENC_Deg_TurretOffset = 127.872;
const double KeENC_Deg_TurretOffset = 135.65; //temp
/* KeROBO_t_MotorTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const double KeROBO_t_MotorTimeoutMs = 30;

/* KeENC_k_TurretEncoderScaler: Scalar multiplied against the encoder read to translate to degrees relative to turret. */
const double KeENC_k_TurretEncoderScaler = -0.025947816048;

/* KeENC_k_LinearSlideEncoderScaler: Scalar multiplied against the encoder read to translate to degrees relative to inches traveled for the linear slide. */
const double KeENC_k_LinearSlideEncoderScaler = 0.001218;

/* K_deg_TurretMinDeltaOL: Minimum delta position change value expected when in OL control.  If this isn't met for a specific amount of time, it will advance to the next state. */
const double K_deg_TurretMinDeltaOL = 0.05;

/* K_t_TurretDebounceTimeout: Debounce time before the turret initialization will advance to the next step. */
const double K_t_TurretDebounceTimeout = 0.5;

/* K_t_TurretOL_Timeout: Max allowed time to be in OL state.  If this is reached, for the turret to be disabled. */
const double K_t_TurretOL_Timeout = 5.0;

/* KeENC_k_ArmPivot: Scalar multiplied against the encoder. */
const double KeENC_k_ArmPivot = 3.44;  

/* KeENC_RPM_IntakeROllers: Finds the speed of the intake rollers. */
const double KeENC_RPM_IntakeRollers = 1.0;

/* KeENC_RPM_Gripper: Finds the speed of the gripper. */
const double KeENC_RPM_Gripper = 1.0;

/* KeENC_Deg_Wrist: Actual position of the wrist, how much we've rotated. */
const double KeENC_Deg_Wrist = -1.16883;

// Manipulator related cals
/* KeMAN_k_ManipulatorNeoCurrentLim: Max allowed current going to each Neo 550 used in the manipulator. */
const double KeMAN_k_ManipulatorNeoCurrentLim = 20;

/* KaMAN_k_ManipulatorTestPower: Test power output for the manipulator controls. ONLY used in test mode!! */
const double KaMAN_k_ManipulatorTestPower[E_MAN_Sz] = {0.25,  // E_MAN_Turret
                                                       0.08,  // E_MAN_ArmPivot
                                                       0.25,  // E_MAN_LinearSlide
                                                       0.05,  // E_MAN_Wrist
                                                       -0.15,  // E_MAN_Gripper
                                                       -0.25,  // E_MAN_IntakeRollers
                                                       1.0}; // E_MAN_IntakeArm

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

/* KaMAN_k_TurretMainIntakePID_Gx: PID gains for the turret control when in main intake.  Intent is to have loose control to allow it to "flop" around. */
const double KaMAN_k_TurretMainIntakePID_Gx[E_PID_CalSz] = { 0.05,      // P Gx
                                                             0.00007,  // I Gx
                                                             0.000000, // D Gx 
                                                             0.4,       // P UL
                                                            -0.4,       // P LL
                                                             0.2,      // I UL
                                                            -0.2,      // I LL
                                                             0.2,       // D UL
                                                            -0.2,       // D LL
                                                             0.4,       // Max upper
                                                            -0.4};      // Max lower

/* KaMAN_k_TurretRotatePID_Gx: PID gains for the turret control when rotating. */
const double KaMAN_k_TurretRotatePID_Gx[E_PID_CalSz] = { 0.05,      // P Gx  .02
                                                         0.00002,  // I Gx
                                                         0.000002, // D Gx 
                                                         0.5,       // P UL
                                                        -0.5,       // P LL
                                                         0.15,      // I UL
                                                        -0.15,      // I LL
                                                         0.2,       // D UL
                                                        -0.2,       // D LL
                                                         0.5,       // Max upper
                                                        -0.5};      // Max lower

/* KaMAN_k_TurretHoldPID_Gx: PID gains for the turret control when attempting to hold still. */
const double KaMAN_k_TurretHoldPID_Gx[E_PID_CalSz] = { 0.05,      // P Gx  .02
                                                         0.00003,  // I Gx
                                                         0.000002, // D Gx 
                                                         0.3,       // P UL
                                                        -0.3,       // P LL
                                                         0.2,      // I UL
                                                        -0.2,      // I LL
                                                         0.2,       // D UL
                                                        -0.2,       // D LL
                                                         0.3,       // Max upper
                                                        -0.3};      // Max lower

/* KaMAN_k_LinearSlidePID_Gx: PID gains for the linear slide control. */
const double KaMAN_k_LinearSlidePID_Gx[E_PID_CalSz] = { 0.7,     // P Gx 45
                                                        0.0000010,  // I Gx
                                                        0.000000, // D Gx 
                                                        0.80,       // P UL
                                                       -0.80,       // P LL
                                                        0.05,      // I UL
                                                       -0.05,      // I LL
                                                        0.2,       // D UL
                                                       -0.2,       // D LL
                                                        0.80,       // Max upper
                                                       -0.80};      // Max lower

/* KaMAN_Deg_TurretAngle: sets turret final positons for each state */
const double KaMAN_Deg_TurretAngle[E_MAN_State_Sz] = {0.0,    // Init
                                                      0.0,    // Driving
                                                      180.0,  // Back High Cube
                                                      180.0,  // Back High Cone
                                                      180.0,  // Back Low Cube
                                                      180.0,  // Back Low Cone
                                                      180.0,  // Mid Transition
                                                      0.0,    // Main Intake
                                                      180.0,  // Floor Intake
                                                      0.0,    // Mid Intake
                                                      0.0,    // Front High Cube
                                                      0.0};   // Front Low Cube

/* KeMAN_DegS_TurretRate: Fast rate that is used in transition for the turret */
const double KeMAN_DegS_TurretRateFast = 3.0;

/* KeMAN_DegS_TurretRateSlow: Slow rate that is used in transition for the turret */
const double KeMAN_DegS_TurretRateSlow = 3.0;

/* KeMAN_DegS_TurretRate: Deadband used to switch to slow ramp rate */
const double KeMAN_Deg_TurretRateDb = 60;

/* KaMAN_Deg_TurretDb: Sets turret dead band */
const double KaMAN_Deg_TurretDb[E_MAN_State_Sz] = {6.0,  // Init
                                                   6.0,  // Driving
                                                   6.0,  // Back High Cube
                                                   6.0,  // Back High Cone
                                                   6.0,  // Back Low Cube
                                                   6.0,  // Back Low Cone
                                                   6.0,  // Mid Transition
                                                   6.0,  // Main Intake
                                                   6.0,  // Floor Intake
                                                   6.0,  // Mid Intake
                                                   6.0,  // Front High Cube
                                                   6.0}; // Front Low Cube

/* KaMAN_Deg_ArmPivotAngle: sets Arm Pivot final positons for each state */
const double KaMAN_Deg_ArmPivotAngle[E_MAN_State_Sz] = {0.0,    // Init
                                                        0.0,    // Driving
                                                        103.60, // Back High Cube
                                                        117.0,  // Back High Cone
                                                        55.60,  // Back Low Cube
                                                        95.89,  // Back Low Cone
                                                         0.00,  // Mid Transition
                                                        32.00,  // Main Intake
                                                        -3.24,  // Floor Intake
                                                        110.08,  // Mid Intake
                                                        115.90, // Front High Cube
                                                        85.00}; // Front Low Cube

/* KeMAN_DegS_ArmPivotFastRate: Sets Arm Pivot transition rate. */
const double KeMAN_DegS_ArmPivotFastRate = 0.28;  // 0.15

/* KeMAN_DegS_ArmPivotSlowRate: Sets Arm Pivot transition rate. */
const double KeMAN_DegS_ArmPivotSlowRate = 0.15;  // 0.15

/* KaMAN_Deg_ArmPivotDb: Sets Arm Pivot dead bandl */
const double KaMAN_Deg_ArmPivotDb[E_MAN_State_Sz] = {4.0,  // Init
                                                     4.0,  // Driving
                                                     4.0,  // Back High Cube
                                                     4.0,  // Back High Cone
                                                     4.0,  // Back Low Cube
                                                     4.0,  // Back Low Cone
                                                     4.0,  // Mid Transition
                                                     4.0,  // Main Intake
                                                     4.0,  // Floor Intake
                                                     4.0,  // Mid Intake
                                                     4.0,  // Front High Cube
                                                     4.0}; // Front Low Cube

/* KaMAN_In_LinearSlidePosition: sets LInear Slide final positons for each state */
const double KaMAN_In_LinearSlidePosition[E_MAN_State_Sz] = {  0.0,   // Init
                                                             -14.0,   // Driving
                                                              -1.77,  // Back High Cube
                                                              11.00,  // Back High Cone
                                                             -13.45,  // Back Low Cube
                                                             -13.0,   // Back Low Cone
                                                             -14.0,   // Mid Transition
                                                               2.7,   // Main Intake
                                                              13.6,   // Floor Intake
                                                              -2.0,   // Mid Intake
                                                              13.513, // Front High Cube
                                                               9.93};  // Front Low Cube

/* KeMAN_t_StateTimeOUt: Sets transition time out. */
const double KeMAN_t_StateTimeOut = 1.5; // Drop-off

/* KeMAN_InS_LinearSlideRate: Sets Linear Slide transition rate. */
const double KeMAN_InS_LinearSlideRate = 9.0; // Drop-off

/* KeMAN_InS_LinearSlideIntakeRate: Sets Linear Slide transition rate. */
const double KeMAN_InS_LinearSlideIntakeRate = 0.6; // Drop-off

/* KaMAN_In_LinearSlideDb: Sets LInear Slide dead band. */
const double KaMAN_In_LinearSlideDb[E_MAN_State_Sz] = {0.5,  // Init
                                                       0.5,  // Driving
                                                       0.5,  // Back High Cube
                                                       0.5,  // Back High Cone
                                                       0.5,  // Back Low Cube
                                                       0.5,  // Back Low Cone
                                                       0.5,  // Mid Transition
                                                       0.5,  // Main Intake
                                                       0.5,  // Floor Intake
                                                       0.5,  // Mid Intake
                                                       0.5,  // Front High Cube
                                                       0.5}; // Front Low Cube

/* KaMAN_Deg_WristAngle: sets Wrist final angle for each state */
const double KaMAN_Deg_WristAngle[E_MAN_State_Sz] = {  0.00,  // Init
                                                      90.00,  // Driving
                                                      0.0,  // Back High Cube
                                                       5.00,  // Back High Cone
                                                      16.35,  // Back Low Cube
                                                      7.34,   // Back Low Cone
                                                      90.00,  // Mid Transition
                                                      18.11,  // Main Intake
                                                      71.96,  // Floor Intake
                                                      -10.30,    // Mid Intake
                                                     -10.0,    // Front High Cube
                                                      -0.6};   // Front Low Cube

/* KeMAN_DegS_WristRate: Sets Wrist transition rate. */
const double KeMAN_DegS_WristRate = 0.45;

/* KaMAN_Deg_WristDb: sets Wrist final angle for each state */
const double KaMAN_Deg_WristDb[E_MAN_State_Sz] = {1.0,  // Init
                                                  1.0,  // Driving
                                                  1.0,  // Back High Cube
                                                  1.0,  // Back High Cone
                                                  1.0,  // Back Low Cube
                                                  1.0,  // Back Low Cone
                                                  1.0,  // Mid Transition
                                                  1.0,  // Main Intake
                                                  1.0,  // Floor Intake
                                                  1.0,  // Mid Intake
                                                  1.0,  // Front High Cube
                                                  1.0}; // Front Low Cube

/* KeMAN_DegS_GripperRate: Sets Gripper transition rate */
const double KeMAN_DegS_GripperRate = 1.0;

/* KeMAN_k_GripperReleaseFast: Sets Gripper release fast power */
const double KeMAN_k_GripperReleaseFast = 0.95;

/* KeMAN_k_GripperReleaseSlow: Sets Gripper release slow power */
const double KeMAN_k_GripperReleaseSlow = 0.15;

/* KeMAN_k_GripperIntake: Sets Gripper intake power */
const double KeMAN_k_GripperIntake = -0.3; //.39

/* KeMAN_k_GripperIntakehold: Sets Gripper intake hold */
const double KeMAN_k_GripperIntakehold = -0.0; 

/* KeMAN_t_GripperOnTm: Amount of time gripper will remain on after it is initially commanded on. */
const double KeMAN_t_GripperOnTm = 0.5;

/* KaMAN_RPM_IntakePower: sets Intake power for each state */
const double KaMAN_RPM_IntakePower[E_MAN_State_Sz] = {  0.0,  // Init
                                                        0.0,  // Driving
                                                        0.0,  // Back High Cube
                                                        0.0,  // Back High Cone
                                                        0.0,  // Back Low Cube
                                                        0.0,  // Back Low Cone
                                                        0.0,  // Mid Transition
                                                       -0.45, // Main Intake
                                                        0.0,  // Floor Intake
                                                        0.0,  // Mid Intake
                                                        0.0,  // Front High Cube
                                                        0.0}; // Front Low Cube

/* KaMAN_e_IntakePneumatics: sets the Pneumatics either true (arm extended) or false (arm retracted) for each state */
const T_MotorControlType KaMAN_e_IntakePneumatics[E_MAN_State_Sz] = {E_MotorRetract,  // Init
                                                                     E_MotorRetract,  // Driving
                                                                     E_MotorRetract,  // Back High Cube
                                                                     E_MotorRetract,  // Back High Cone
                                                                     E_MotorRetract,  // Back Low Cube
                                                                     E_MotorRetract,  // Back Low Cone
                                                                     E_MotorRetract,  // Mid Transition
                                                                     E_MotorExtend,   // Main Intake
                                                                     E_MotorRetract,  // Floor Intake
                                                                     E_MotorRetract,  // Mid Intake
                                                                     E_MotorRetract,  // Front High Cube
                                                                     E_MotorRetract}; // Front Low Cube

/* KaMAN_e_ControllingTable: Table that contains the commanded state of the manipulator and intake based on the current attained state and schedueld state. */
const TeMAN_ManipulatorStates KaMAN_e_ControllingTable[E_MAN_State_Sz][E_MAN_State_Sz] =  // [Sched][Attnd]
  {
    {E_MAN_Init,    E_MAN_Init,          E_MAN_BackHighCube,  E_MAN_BackHighCone,  E_MAN_BackLowCube,   E_MAN_BackLowCone,   E_MAN_MidTransition, E_MAN_MainIntake, E_MAN_FloorIntake,   E_MAN_MidIntake,     E_MAN_FrontHighCube, E_MAN_FrontLowCube},  // Sched - Init
    {E_MAN_Driving, E_MAN_Driving,       E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,    E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Driving
    {E_MAN_Driving, E_MAN_MidTransition, E_MAN_BackHighCube,  E_MAN_BackHighCube,  E_MAN_BackHighCube,  E_MAN_BackHighCube,  E_MAN_BackHighCube,  E_MAN_Driving,    E_MAN_BackHighCube,  E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Back High Cube
    {E_MAN_Driving, E_MAN_MidTransition, E_MAN_BackHighCone,  E_MAN_BackHighCone,  E_MAN_BackHighCone,  E_MAN_BackHighCone,  E_MAN_BackHighCone,  E_MAN_Driving,    E_MAN_BackHighCone,  E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Back High Cone
    {E_MAN_Driving, E_MAN_MidTransition, E_MAN_BackLowCube,   E_MAN_BackLowCube,   E_MAN_BackLowCube,   E_MAN_BackLowCube,   E_MAN_BackLowCube,   E_MAN_Driving,    E_MAN_BackLowCube,   E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Back Low Cube
    {E_MAN_Driving, E_MAN_MidTransition, E_MAN_BackLowCone,   E_MAN_BackLowCone,   E_MAN_BackLowCone,   E_MAN_BackLowCone,   E_MAN_BackLowCone,   E_MAN_Driving,    E_MAN_BackLowCone,   E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Back Low Cone
    {E_MAN_Driving, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_Driving,    E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Mid Transition
    {E_MAN_Driving, E_MAN_MainIntake,    E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_Driving,       E_MAN_MainIntake, E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Main Intake
    {E_MAN_Driving, E_MAN_MidTransition, E_MAN_FloorIntake,   E_MAN_FloorIntake,   E_MAN_FloorIntake,   E_MAN_FloorIntake,   E_MAN_FloorIntake,   E_MAN_Driving,    E_MAN_FloorIntake,   E_MAN_Driving,       E_MAN_Driving,       E_MAN_Driving},       // Sched - Floor Intake
    {E_MAN_Driving, E_MAN_MidIntake,     E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,    E_MAN_MidTransition, E_MAN_MidIntake,     E_MAN_MidIntake,     E_MAN_MidIntake},     // Sched - Mid Intake
    {E_MAN_Driving, E_MAN_FrontHighCube, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,    E_MAN_MidTransition, E_MAN_FrontHighCube, E_MAN_FrontHighCube, E_MAN_FrontHighCube}, // Sched - Front High Cube
    {E_MAN_Driving, E_MAN_FrontLowCube,  E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_MidTransition, E_MAN_Driving,       E_MAN_Driving,    E_MAN_MidTransition, E_MAN_FrontLowCube,  E_MAN_FrontLowCube,  E_MAN_FrontLowCube}   // Sched - Front Low Cube
  };

/* Ball handler (BH) cals: */
/* K_BH_IntakePower: Amount of power to apply to intake wheels.  Must be 0 to 1. */
const double K_BH_IntakePower = 0.7;

/* K_BH_ElevatorPowerUp: Amount of power to apply to elevator band when commanded up.  Must be 0 to 1. */
const double K_BH_ElevatorPowerUp = 0.9;

/* K_BH_ElevatorPowerDwn: Amount of power to apply to elevator band when commanded down.  Must be -1 to 0. */
const double K_BH_ElevatorPowerDwn = -0.9;

/* K_BH_LauncherMinCmndSpd: Min speed for launcher control.  Below this speed, launcher will transition to 0 power.  Also 
   acts as the indicator for allowing the elevator to run.  If the commanded launcher speed is above this threshold, then 
   we will use the upper ball sensor to determine when to allow the elevator to proceed based on launcher RPM deadband.  */
const double K_BH_LauncherMinCmndSpd = 10;

/* K_BH_LauncherPID_Gx: PID gains for the launcher. */
const double K_BH_LauncherPID_Gx[E_PID_SparkMaxCalSz] = { 0.00055,  // kP
                                                          0.000001, // kI
                                                          0.0,      // kD
                                                          0.0,      // kIz
                                                          0.0,      // kFF
                                                          1.0,      // kMaxOutput
                                                         -1.0,      // kMinOutput
                                                          0.0,      // kMaxVel
                                                          0.0,      // kMinVel
                                                         55.0,      // kMaxAcc
                                                          0.0};     // kAllErr

/* K_BH_LauncherSpeedAxis: Launcher speed axis for K_BH_LauncherSpeed.  Distance is in the unit from the camera.  Comments reflect actual measured distance. */
const double K_BH_LauncherSpeedAxis[4] = {2.52,  // 6 ft 6in
                                          3.60,  // 8 ft 6in
                                          4.51,  // 10 ft 6in
                                          13.5}; // 17 ft

/* K_BH_LauncherSpeed: Launcher speed necessary for ball to reach target based on the estimated distance from the camera. */
const double K_BH_LauncherSpeed[4] = {2000,  // 6 ft 6in 3300
                                      2300,  // 8 ft 6in 3650
                                      2600,  // 10 ft 6in 3900
                                      3200}; // 17 ft 5000

/* K_BH_LauncherManualDb: Deadband around the manual ball launcher axis. */
const double K_BH_LauncherManualDb = 0.1;

/* K_BH_LauncherManualHi: Manual speed single point.4600 */
const double K_BH_LauncherManualHi = 3300;

/* K_BH_LauncherManualLo: Manual speed single point. */
const double K_BH_LauncherManualLo = 1600;

/* K_BH_LauncherSpeedDb: Deadband around the commanded launcher speed (in RPM).  
                         Used to indicate when a ball can be launched. */
const double K_BH_LauncherSpeedDb = 40;



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
const double KeENC_Deg_SD_WheelOffsetAngle[E_RobotCornerSz] = {152.578125,   // E_FrontLeft
                                                               212.783203,   // E_FrontRight 
                                                               118.740234,   // E_RearLeft
                                                               76.289063};  // E_RearRight

/* KeENC_k_WheelOffsetAnglePractieBot: Offset angle for each respective corder of the swerve drive wheel.  This is the angle 
   reading from the absolute encoder that is indicated in order for the wheel to point straight.  For practice bot only. */
const double KeENC_k_WheelOffsetAnglePractieBot[E_RobotCornerSz] = { -179.4,  // E_FrontLeft 1.3  -176
                                                                    -16.1,  // E_FrontRight 163.5
                                                                     52.9,   // E_RearLeft 230.8  -127.1
                                                                     96.9};  // E_RearRight 282.0

/* K_SD_WheelGx: Gain multiplied by each calculated desired speed.  Intended to account for variation in wheel size. */
const double K_SD_WheelGx[E_RobotCornerSz] = {-1.0,  // E_FrontLeft
                                              -1.0,  // E_FrontRight 
                                              -1.0,  // E_RearLeft // +
                                              -1.0}; // E_RearRight 

/* K_SD_MinGain: Min gain applied to the wheel speed for swerve drive. */
const double K_SD_MinGain = 0.2;

/* K_SD_MaxGain: Max gain allowed for swerve drive control. */
const double K_SD_MaxGain = 0.7;


/* Ke_SD_AutoCorrectPID_Gx: PID gains for the auto correct.  PID control is within the RoboRio.  */
const double Ke_SD_AutoCorrectPID_Gx[E_PID_CalSz] = { 40.0,      // P Gx  75
                                                       0.5,  // I Gx 0.03
                                                       0.0005, // D Gx 0.0005
                                                      300.0,       // P UL 0.6
                                                     -300.0,       // P LL -0.4
                                                     300.0,      // I UL 0.12
                                                    -300.0,      // I LL -0.12
                                                      1.0,       // D UL 0.5
                                                     -1.0,       // D LL -0.5
                                                     400.0,       // Max upper 0.9
                                                    -400.0};      // Max lower -0.9

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
                                                              150.0,     // kMaxAcc
                                                               0.0};     // kAllErr

/* K_SD_WheelAnglePID_Gx: PID gains for the angle of the swerve drive wheels.  PID control is within the RoboRio.  */
const double K_SD_WheelAnglePID_Gx[E_PID_CalSz] = { 0.0035,      // P Gx  0.002
                                                               0.000001,  // I Gx 0.000001
                                                               0.000005, // D Gx 0.0000005
                                                               1.0,       // P UL 0.6
                                                              -1.0,       // P LL -0.4
                                                               0.15,      // I UL 0.12
                                                              -0.15,      // I LL -0.12
                                                               1.0,       // D UL 0.5
                                                              -1.0,       // D LL -0.5
                                                               1.0,       // Max upper 0.9
                                                              -1.0};      // Max lower -0.9

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

/* K_SD_DesiredDriveSpeed: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double K_SD_DesiredDriveSpeed[20] = {-1.00,  //-0.95
                                           -0.88,  //-0.85
                                           -0.77,  //-0.75
                                           -0.66,  //-0.65
                                           -0.55,  //-0.55
                                           -0.44,  //-0.45
                                           -0.33,  //-0.35
                                           -0.22,  //-0.25
                                           -0.11,  //-0.15
                                            0.00,  //-0.10
                                            0.00,  // 0.10
                                            0.11,  // 0.15
                                            0.22,  // 0.25
                                            0.33,  // 0.35
                                            0.44,  // 0.45
                                            0.55,  // 0.55
                                            0.66,  // 0.65
                                            0.77,  // 0.75
                                            0.88,  // 0.85
                                            1.00}; // 0.95

/* RotateDeadBand: Check Rotation value approx 0 */
const double K_SD_RotateDeadBand = 0.05; 

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
/* Upper Targeting Cals (UT) */
/* K_ADAS_UT_LightDelayTIme - Amount of time wait for the camera to have sufficent light before proceeding. [Seconds] */
const double K_ADAS_UT_LightDelayTIme = 0.060;

/* K_ADAS_UT_LostTargetGx - When the camera has lost the target, the previous error value will be used,
   but multiplied against this gain so that we don't go too far before getting another good value. */
const double K_ADAS_UT_LostTargetGx = 0.25;

/* K_ADAS_UT_NoTargetError - When we haven't seen anything from the camera, take a guess.  This will 
   be the percieved error value until we see something good. */
const double K_ADAS_UT_NoTargetError = 2;

/* K_ADAS_UT_DebounceTime - Debounce time to hold a given state before preceding to next step. [Seconds] */
const double K_ADAS_UT_DebounceTime = 0.030;

/* K_ADAS_UT_AllowedLauncherError - Amount of error allowed in launcher speed before attempting to launch balls. [RPM] */
const double K_ADAS_UT_AllowedLauncherError = 100;

/* K_ADAS_UT_AllowedLauncherTime - Amount of time to remain in auto elevator mode.  For auton only.  Each time a ball is detected, this timer will reset. [Seconds] */
const double K_ADAS_UT_AllowedLauncherTime = 2;

/* K_ADAS_UT_RotateDeadbandAngle: Deadband angle for upper targeting */
const double K_ADAS_UT_RotateDeadbandAngle = 1.2; //0.9

/* K_ADAS_UT_TargetVisionAngle: This is the desired target angle for the auto vision targeting.  This is due to the offset of the camera. For 2020 - 3.3 */
const double K_ADAS_UT_TargetVisionAngle = -5.0;

/* K_ADAS_BT_LightDelayTIme - Amount of time wait for the camera to have sufficent light before proceeding. [Seconds] */
const double K_ADAS_BT_LightDelayTIme = 0.060;

/* K_ADAS_BT_LostTargetGx - When the camera has lost the target, the previous error value will be used,
   but multiplied against this gain so that we don't go too far before getting another good value. */
const double K_ADAS_BT_LostTargetGx = 0.080;

/* K_ADAS_BT_NoTargetError - When we haven't seen anything from the camera, take a guess.  This will 
   be the percieved error value until we see something good. */
const double K_ADAS_BT_NoTargetError = 1.2;

/* K_ADAS_BT_DebounceTime - Debounce time to hold a given state before preceding to next step. [Seconds] */
const double K_ADAS_BT_DebounceTime = 0.020;

/* K_ADAS_BT_TimeOut - If the ball can't be located in this amount of time, abort out of BT. [Seconds] */
const double K_ADAS_BT_TimeOut = 1.5;

/* K_ADAS_BT_RotateDeadbandAngle: Deadband angle for ball targeting */
const double K_ADAS_BT_RotateDeadbandAngle = 1.8;

/* K_ADAS_BT_TargetVisionAngle: This is the desired target angle for the auto ball vision targeting.  This is due to the offset of the camera. */
const double K_ADAS_BT_TargetVisionAngle = 2.0;

/* K_ADAS_BT_DriveTimeAxis: This is the estimated distance from the camera that will be scaled against the drive time table K_ADAS_BT_DriveTime. [meters] */
const double K_ADAS_BT_DriveTimeAxis[6] = {0,
                                          1,
                                          2,
                                          3,
                                          4,
                                          5};

/* K_ADAS_BT_DriveTime: This is the amount of time to drive forward to capture the ball based on the estimated distance. [seconds] */
const double K_ADAS_BT_DriveTime[6] = {1.5,
                                       1.5,
                                       1.5,
                                       1.5,
                                       1.5,
                                       1.5};

/* K_ADAS_BT_MaxTimeToWaitForCamera: This is the max amount of time we will wait for a valid distance from the camera. [Seconds] */
const double K_ADAS_BT_MaxTimeToWaitForCamera = 0.8;

/* K_ADAS_BT_SettleTimeBeforeDriveForward: This is the amount of time to allow for settling prior to driving forward to pickup the ball.  This MUST be below K_ADAS_BT_TimedOutDriveForward [Seconds] */
const double K_ADAS_BT_SettleTimeBeforeDriveForward = 0.03;

/* K_ADAS_BT_TimedOutDriveForward: This is the default drive forward time when we have waited too long for the camera. [Seconds] */
const double K_ADAS_BT_TimedOutDriveForward = 4.0;

/* K_ADAS_BT_DriveForwardPct: This is the percent of swerve drive control to go forward to pickup the ball. */
const double K_ADAS_BT_DriveForwardPct = -0.40;


/* K_ADAS_DM_BlindShotTime: This is the amount of time to remain in blind shoot. [Seconds] */
const double K_ADAS_DM_BlindShotTime = 4.0;

/* K_ADAS_DM_BlindShotElevator: This is the amount of time to remain in blind shoot. [Seconds] */
const double K_ADAS_DM_BlindShotElevator = 0.7;

/* K_ADAS_DM_BlindShotIntake: This is the amount of intake power to request when in blind shoot. [pct] */
const double K_ADAS_DM_BlindShotIntake = 0.7;

/* K_ADAS_DM_BlindShotLauncher: This is the speed the launcher will be shot at while in shoot. [RPM] */
const double K_ADAS_DM_BlindShotLauncherLow = 1600;

/* K_ADAS_DM_BlindShotLauncher: This is the speed the launcher will be shot at while in shoot. [RPM] */
const double K_ADAS_DM_BlindShotLauncherHigh = 4000; //4000 is temporary make sure to tune later

/* K_ADAS_DM_DriveTimeShort: This is the short drive forward time. [Seconds] */
const double K_ADAS_DM_DriveTimeShort = 4.5; 

/* K_ADAS_DM_DriveTimeLong: This is the default drive forward time. [Seconds] */
const double K_ADAS_DM_DriveTimeLong = 5.5;

/* K_ADAS_DM_DriveFWD_Pct: This is the default drive forward Pct. [Pct] */
const double K_ADAS_DM_DriveFWD_Pct = -0.3;

/* KeADAS_t_DM_DriveTimeFar: This is the drive far time. [Seconds] */
const double KeADAS_t_DM_DriveTimeFar = 11.5;

/* KeADAS_Pct_DM_DriveFWD_Far: This is the default drive forward Pct. [Pct] */
const double KeADAS_Pct_DM_DriveFWD_Far = -0.5;

/* KeADAS_t_DM_RevDriveTime: Time to drive in reverse. [Seconds] */
const double KeADAS_t_DM_RevDriveTime = 1.0;

/* KeADAS_Pct_DM_RevDrive: This is the reverse drive  Pct. [Pct] */
const double KeADAS_Pct_DM_RevDrive = 0.5;

/* K_ADAS_DM_DriveREV_Pct: This is the default drive in reverse Pct. [Pct] */
const double K_ADAS_DM_DriveREV_Pct = -0.2;

/* K_ADAS_DM_RotateDebounceTime: This is the debounce time for the DM rotate state. [seconds] */
const double K_ADAS_DM_RotateDebounceTime = 0.02;

/* K_ADAS_DM_RotateDeadbandAngle: This is the deband angle for the DM rotate state. [degrees] */
const double K_ADAS_DM_RotateDeadbandAngle = 1.8;

/* K_ADAS_DM_XY_Deadband: This is the deband position for the DM XY drive state. [meters] */
const double K_ADAS_DM_XY_Deadband = 0.1;

/* KeADAS_Deg_DM_AutoBalanceDb: This is the deband angle for the DM auto balance state. [degrees] */
const double KeADAS_Deg_DM_AutoBalanceDb = 10.0;

/* KeADAS_t_DM_AutoBalanceDb: This is the debounce time for the DM auto balance state. [seconds] */
const double KeADAS_t_DM_AutoBalanceDb = 0.06;

/* KeADAS_t_DM_AutoBalanceHold: This is the amount of time for the DM auto balance to hold. [seconds] */
const double KeADAS_t_DM_AutoBalanceHold = 10;

/* KeADAS_k_DM_AutoBalanceFastPID: This is the PID gains for the auto balance. */
const double KeADAS_k_DM_AutoBalanceFastPID[E_PID_CalSz] = { 0.037,       // P Gx
                                                             0.0000007,    // I Gx
                                                             0.00001,      // D Gx
                                                             0.8,       // P UL
                                                            -0.8,       // P LL
                                                             0.03,      // I UL
                                                            -0.03,      // I LL
                                                              0.5,       // D UL
                                                             -0.5,       // D LL
                                                              0.6,       // Max upper
                                                             -0.6};      // Max lower

/* KeADAS_k_DM_AutoBalanceSlowPID: This is the PID gains for the auto balance. */
const double KeADAS_k_DM_AutoBalanceSlowPID[E_PID_CalSz] = { 0.037,       // P Gx
                                                             0.0000007,    // I Gx
                                                             0.00001,      // D Gx
                                                             0.8,       // P UL
                                                            -0.8,       // P LL
                                                             0.03,      // I UL
                                                            -0.03,      // I LL
                                                              0.5,       // D UL
                                                             -0.5,       // D LL
                                                              0.60,       // Max upper
                                                             -0.60};      // Max lower

/* KeADAS_t_DM_TagCenteringDb: This is the deband position for the DM drive state auto centering. [sec] */
const double KeADAS_t_DM_TagCenteringDb = 0.1;

/* KeADAS_Deg_DM_AutoMountDetect: Amount of angle to indicate when bot has mounted onto the charge station. [degrees] */
const double KeADAS_Deg_DM_AutoMountDetect = 5;

/* KeADAS_t_DM_AutoMountDb: Debounce time for auto mount. [sec] */
const double KeADAS_t_DM_AutoMountDb = 0.5;

/* KeADAS_t_DM_AutoMountOnlyDb: Debounce time for auto mount only, not used for mount/dismount. [sec] */
const double KeADAS_t_DM_AutoMountOnlyDb = 1.0;

/* KeADAS_t_DM_AutoMountRevDb: Debounce time for auto mount. [sec] */
const double KeADAS_t_DM_AutoMountRevDb = 0.85;

/* KeADAS_Pct_DM_AutoMountPwr: Power command when in auto mount. [pct] */
const double KeADAS_Pct_DM_AutoMountPwr = -0.99;

/* KeADAS_t_DM_StopTm: Amount of time to have the robot stopped. [sec] */
const double KeADAS_t_DM_StopTm = 0.1;

/* Motion profiles for DM: */
#include "MotionProfiles/Red1Ang.hpp"
#include "MotionProfiles/Red1T.hpp"
#include "MotionProfiles/Red1X.hpp"
#include "MotionProfiles/Red1Y.hpp"

#include "MotionProfiles/Red2Ang.hpp"
#include "MotionProfiles/Red2T.hpp"
#include "MotionProfiles/Red2X.hpp"
#include "MotionProfiles/Red2Y.hpp"

#include "MotionProfiles/Red3Ang.hpp"
#include "MotionProfiles/Red3T.hpp"
#include "MotionProfiles/Red3X.hpp"
#include "MotionProfiles/Red3Y.hpp"

#include "MotionProfiles/StartToGameP1.hpp"
#include "MotionProfiles/StartToGameP2.hpp"
#include "MotionProfiles/StartToGameP3.hpp"
#include "MotionProfiles/StartToGameP4.hpp"

#include "MotionProfiles/TestPath1.hpp"

/*  Rotation calibrations */
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
const double K_k_AutonX_PID_Gx[E_PID_CalSz] = { 0.18,       // P Gx
                                                0.000001,    // I Gx
                                                0.00012,      // D Gx
                                                0.8,       // P UL
                                               -0.8,       // P LL
                                                0.05,      // I UL
                                               -0.05,      // I LL
                                                0.5,       // D UL
                                               -0.5,       // D LL
                                                1.0,       // Max upper
                                               -1.0};      // Max lower

const double K_k_AutonY_PID_Gx[E_PID_CalSz] = { 0.18,       // P Gx
                                                0.000001,    // I Gx
                                                0.00012,      // D Gx
                                                0.8,       // P UL
                                               -0.8,       // P LL
                                                0.05,      // I UL
                                               -0.05,      // I LL
                                                0.5,       // D UL
                                               -0.5,       // D LL
                                                1.0,       // Max upper
                                               -1.0};      // Max lower

// #include "MotionProfiles/K_BarrelRacing_V55A25_T.hpp"
// #include "MotionProfiles/K_BarrelRacing_V55A25_X.hpp"
// #include "MotionProfiles/K_BarrelRacing_V55A25_Y.hpp"
// #include "MotionProfiles/K_BarrelRacing_V75A30_T.hpp"
// #include "MotionProfiles/K_BarrelRacing_V75A30_X.hpp"
// #include "MotionProfiles/K_BarrelRacing_V75A30_Y.hpp"
// #include "MotionProfiles/K_BarrelRacing_V95A35_T.hpp"
// #include "MotionProfiles/K_BarrelRacing_V95A35_X.hpp"
// #include "MotionProfiles/K_BarrelRacing_V95A35_Y.hpp"

// #include "MotionProfiles/K_Bounce_V55A25_T.hpp"
// #include "MotionProfiles/K_Bounce_V55A25_X.hpp"
// #include "MotionProfiles/K_Bounce_V55A25_Y.hpp"
// #include "MotionProfiles/K_Bounce_V75A30_T.hpp"
// #include "MotionProfiles/K_Bounce_V75A30_X.hpp"
// #include "MotionProfiles/K_Bounce_V75A30_Y.hpp"
// #include "MotionProfiles/K_Bounce_V95A35_T.hpp"
// #include "MotionProfiles/K_Bounce_V95A35_X.hpp"
// #include "MotionProfiles/K_Bounce_V95A35_Y.hpp"

// #include "MotionProfiles/K_Slalom_V55A25_T.hpp"
// #include "MotionProfiles/K_Slalom_V55A25_X.hpp"
// #include "MotionProfiles/K_Slalom_V55A25_Y.hpp"
// #include "MotionProfiles/K_Slalom_V75A30_T.hpp"
// #include "MotionProfiles/K_Slalom_V75A30_X.hpp"
// #include "MotionProfiles/K_Slalom_V75A30_Y.hpp"
// #include "MotionProfiles/K_Slalom_V95A35_T.hpp"
// #include "MotionProfiles/K_Slalom_V95A35_X.hpp"
// #include "MotionProfiles/K_Slalom_V95A35_Y.hpp"
// #include "MotionProfiles/K_Slalom_V125A50_T.hpp"
// #include "MotionProfiles/K_Slalom_V125A50_X.hpp"
// #include "MotionProfiles/K_Slalom_V125A50_Y.hpp"




