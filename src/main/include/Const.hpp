#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired test state here: COMP (no test), BallHandlerTest, LiftXY_Test, DriveMotorTest, WheelAngleTest, ADAS_UT_Test, ADAS_BT_Test
#define COMP
// Define the bot type: CompBot, PracticeBot
#define PracticeBot

// RoboRio controller execution time
const double C_ExeTime = 0.02; // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms

// Amount of time for end game
const double C_End_game_time = 30;

// Numerical constants
const double C_RadtoDeg = 57.2957795;
const double C_Deg2Rad = 0.017453292519943295;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;

static const double C_EncoderToAngle = 360; // Raw output of PWM encoder to degrees
static const double C_VoltageToAngle = 72.0; // Gain that converts the measured voltage of the absolute encoder to an equivalent angle in degrees. (practice bot only)


// CAN Device IDs:
static const int C_PDP_ID = 0;
static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID  = 5, rearLeftDriveDeviceID  = 6, rearRightSteerDeviceID  = 7, rearRightDriveDeviceID  = 8;
static const int rightShooterID = 10, leftShooterID = 9;
static const int C_liftYD_ID = 11;
static const int C_liftXD_ID = 12;
static const int C_elevatorID = 13;
static const int C_intakeID = 14;
static const int C_turretID = 15;
static const int C_i_Gyro = 16;

// Analog IDs:
static const int C_MagEncoderFL_ID = 2, C_MagEncoderFR_ID = 1, C_MagEncoderRL_ID = 3, C_MagEncoderRR_ID = 0;

// DIO IDs:
static const int C_XY_LimitSwitch_ID = 4, C_XD_LimitSwitch_ID = 6, C_IR_Sensor_ID = 3, C_CameraLightControl_ID = 7;
static const int C_LowerBallSensorID = 5;
static const int C_TurretSensorID = 9;


// PWM IDs:
static const int C_VanityLight_ID = 0;


// Vision Cals:
// cals for top target cam
/* K_VisionHeight: Height of the camera relative to ground. */
const units::meter_t K_VisionHeight[E_CamLocSz] = {0.795_m,  // 795 mm to camera lense  -> top
                                                   0.367_m}; //                         -> bottom

/* K_VisionTargetHeight: Height of the target relative to ground. */
const units::meter_t K_VisionTargetHeight[E_CamLocSz] = {2.58_m,  // bottom of tape to carpet  -> top
                                                         0.12_m};  // radius of the ball in cm -> bottom

/* K_VisionCameraPitch: Pitch of the camera relative to the ground. */
const units::radian_t K_VisionCameraPitch[E_CamLocSz] = {15_deg,  // camera on a 75 degree tilt  -> top
                                                         50_deg}; //                             -> bottom

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
/* K_t_GyroTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const int K_t_GyroTimeoutMs = 30;

// Encoder / speed calculation related cals
const double K_ReductionRatio = 8.31;
const double K_WheelCircufrence = 0.3191764; // Circumferance of wheel, in inches


// Turret cals
/* K_t_TurretTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const double K_t_TurretTimeoutMs = 30;

/* K_Pct_TurretOpenLoopCmnd: Percent motor command sent to turrent when in open loop control. */
const double K_Pct_TurretOpenLoopCmnd = 0.1;

/* K_k_TurretEncoderScaler: Scalar multiplied against the encoder read to translate to degrees relative to turret. */
const double K_k_TurretEncoderScaler = 0.025947816048;

/* K_deg_TurretMinDeltaOL: Minimum delta position change value expected when in OL control.  If this isn't met for a specific amount of time, it will advance to the next state. */
const double K_deg_TurretMinDeltaOL = 0.05;

/* K_t_TurretDebounceTimeout: Debounce time before the turret initialization will advance to the next step. */
const double K_t_TurretDebounceTimeout = 0.5;

/* K_t_TurretOL_Timeout: Max allowed time to be in OL state.  If this is reached, for the turret to be disabled. */
const double K_t_TurretOL_Timeout = 5.0;


// Lift related cals
/* K_LiftRampRateYD: Per loop revolutions of the motor allowed for the YD position. */
const double K_LiftRampRateYD[E_Lift_State_Sz][E_LiftIterationSz] = 
  {
    {1.25, 1.25},  // E_S0_BEGONE
    {1.35, 1.25},  // E_S2_lift_down_YD
    {1.25, 1.25},  // E_S3_move_forward_XD
    {1.15, 1.15},  // E_S4_stretch_up_YD
    {1.25, 1.25},  // E_S5_more_forward_XD
    {1.25, 1.25},  // E_S6_lift_up_more_YD
    {1.25, 1.25},  // E_S7_move_back_XD
    {1.25, 1.25},  // E_S8_more_down_some_YD
    {1.25, 1.25},  // E_S9_back_rest_XD
    {1.00, 1.00},  // E_S10_final_YD
    {1.25, 1.25}   // E_S11_final_OWO
  };

/* K_LiftRampRateXD: Per loop revolutions of the motor allowed for the XD position. */
const double K_LiftRampRateXD[E_Lift_State_Sz][E_LiftIterationSz] = 
  {
    {1.05, 1.05},  // E_S0_BEGONE
    {1.05, 1.05},  // E_S2_lift_down_YD
    {1.05, 1.05},  // E_S3_move_forward_XD
    {1.05, 1.05},  // E_S4_stretch_up_YD
    {1.05, 1.05},  // E_S5_more_forward_XD
    {1.05, 1.05},  // E_S6_lift_up_more_YD
    {1.15, 0.50},  // E_S7_move_back_XD
    {1.05, 1.05},  // E_S8_more_down_some_YD
    {1.05, 1.05},  // E_S9_back_rest_XD
    {1.05, 1.05},  // E_S10_final_YD
    {1.05, 1.05}   // E_S11_final_OWO
  };

const double K_lift_S2_YD = 8; //initial lift of the robot
const double K_lift_S3_YD = 8; //stays the same
const double K_lift_S4_YD = 26; //Move YD off of hooks
const double K_lift_S5_YD = 26; //stays the same
const double K_lift_S6_YD = 38; //lower YD below the rung
const double K_lift_S7_YD = 165; //
const double K_lift_S8_YD = 210; //
const double K_lift_S9_YD = 210; //stays the same
const double K_lift_S10_YD = 170; //
const double K_lift_S11_YD = 140; //

const double K_lift_max_YD = 210; //max allowed travel distance of YD
const double K_lift_min_YD = 0; //it crunch
const double K_lift_enable_auto_YD = 150; //distance the lift must be above to allow the driver to enable the auto climb
const double K_lift_deadband_YD = 1.1; //it's a deadband for the y lift yeah
const double K_lift_driver_up_rate_YD = 1.8; // This is the amount of traversal added per loop (0.02 sec)
const double K_lift_driver_down_rate_YD = 0.3; // This is the amount of traversal added per loop (0.02 sec)
const double K_lift_driver_manual_up_YD = 0.25; // Manual override power
const double K_lift_driver_manual_down_YD = -0.25; // Manual override power
const double K_lift_autoResetDown_YD = -0.20; // Auto reset power

const double K_lift_S3_XD = 30; //move XD onto the rungs
const double K_lift_S4_XD = 30; //stays the same
const double K_lift_S5_XD = 32; //tilt the robot
const double K_lift_S6_XD = 34; //connect YD with the upper rungs
const double K_lift_S7_XD = 133; //
const double K_lift_S8_XD = 133; //stays the same
const double K_lift_S9_XD = 122; //
const double K_lift_S10_XD = 122; //stays the same
const double K_lift_S11_XD = 0; //

const double K_lift_max_XD = 133; //max allowed travel distance of XD
const double K_lift_min_XD = 0; //we don't want XD to past this or it crunch
const double K_lift_deadband_XD = 0.7; //it's a deadband for the x lift yeah
const double K_lift_driver_manual_forward_XD = 0.15; // Manual override power
const double K_lift_driver_manual_back_XD = -0.15; // Manual override power

const double K_Lift_deadband_timer = 0.035; //keep the deadband for a certain amount of time [seconds]

const double K_LiftPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                   0.000001, // kI
                                                   0.002000, // kD
                                                   0.0,      // kIz
                                                   0.0,      // kFF
                                                   1.0,      // kMaxOutput
                                                  -1.0,      // kMinOutput
                                                   1.05,     // kMaxVel - 0.93
                                                   0.5,      // kMinVel
                                                   0.0,      // kMaxAcc
                                                   0.0};     // kAllErr



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

/* K_SD_WheelOffsetAngle: Offset angle for each respective corder of the swerve drive wheel.  This is the angle 
   reading from the absolute encoder that is indicated in order for the wheel to point straight. */
const double K_SD_WheelOffsetAngle[E_RobotCornerSz] = {174.527239,   // E_FrontLeft
                                                       128.487963,   // E_FrontRight 
                                                        33.112801,   // E_RearLeft
                                                       250.813891};  // E_RearRight

/* K_SD_WheelOffsetAnglePractieBot: Offset angle for each respective corder of the swerve drive wheel.  This is the angle 
   reading from the absolute encoder that is indicated in order for the wheel to point straight.  For practice bot only. */
const double K_SD_WheelOffsetAnglePractieBot[E_RobotCornerSz] = {-177.9,  // E_FrontLeft 1.3  -176
                                                                 -16.1,  // E_FrontRight 163.5
                                                                 -127.1,   // E_RearLeft 230.8
                                                                 96.9};  // E_RearRight 282.0

/* K_SD_WheelGx: Gain multiplied by each calculated desired speed.  Intended to account for variation in wheel size. */
const double K_SD_WheelGx[E_RobotCornerSz] = {-1.0,  // E_FrontLeft
                                              -1.0,  // E_FrontRight 
                                              1.0,  // E_RearLeft
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
const double K_SD_WheelAnglePID_Gx[E_PID_CalSz] = { 0.0035,   // P Gx
                                                    0.000001, // I Gx
                                                    0.000005, // D Gx
                                                    1.0,      // P UL
                                                   -1.0,      // P LL
                                                    0.1500,   // I UL
                                                   -0.1500,   // I LL
                                                    1.0,      // D UL
                                                   -1.0,      // D LL
                                                    1.0,      // Max upper
                                                   -1.0};     // Max lower

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
const double K_ADAS_DM_DriveFWD_Pct = 0.2;

/* K_ADAS_DM_DriveREV_Pct: This is the default drive in reverse Pct. [Pct] */
const double K_ADAS_DM_DriveREV_Pct = -0.2;

/* K_ADAS_DM_RotateDebounceTime: This is the debounce time for the DM rotate state. [seconds] */
const double K_ADAS_DM_RotateDebounceTime = 0.02;

/* K_ADAS_DM_RotateDeadbandAngle: This is the deband angle for the DM rotate state. [degrees] */
const double K_ADAS_DM_RotateDeadbandAngle = 1.8;

/* K_ADAS_DM_XY_Deadband: This is the deband position for the DM XY drive state. [meters] */
const double K_ADAS_DM_XY_Deadband = 0.1;

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




