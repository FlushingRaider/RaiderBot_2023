/*
Team 5561 Shuffleboard PID code

Writen by - Chris 2024

*/

#include <frc/shuffleboard/Shuffleboard.h>
#include "Shuffleboard.hpp"
#include <networktables/NetworkTable.h>


void shuffleboard_init(){
double FrontLeft_P = frc::Shuffleboard::GetTab("Front Left")
.Add("P", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_I = frc::Shuffleboard::GetTab("Front Left")
.Add("I", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_D = frc::Shuffleboard::GetTab("Front Left")
.Add("D", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_P_LL = frc::Shuffleboard::GetTab("Front Left")
.Add("P LL", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_P_UL = frc::Shuffleboard::GetTab("Front Left")
.Add("I UL", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_I_LL = frc::Shuffleboard::GetTab("Front Left")
.Add("I LL", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_I_UL = frc::Shuffleboard::GetTab("Front Left")
.Add("D UL", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_D_LL = frc::Shuffleboard::GetTab("Front Left")
.Add("D LL", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_D_UL = frc::Shuffleboard::GetTab("Front Left")
.Add("D UL", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_MaxUpper = frc::Shuffleboard::GetTab("Front Left")
.Add("Max Upper", 0).GetEntry()->GetDouble(0.0);
double FrontLeft_MaxLower = frc::Shuffleboard::GetTab("Front Left")
.Add("Max Lower", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_P = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerP", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_I = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerI", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_D = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerD", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_P_LL = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerP LL", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_P_UL = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerP UL", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_I_LL = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerI LL", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_I_UL = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerI UL", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_D_LL = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerD LL", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_D_UL = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerD UL", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_MaxUpper = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerMax Upper", 0).GetEntry()->GetDouble(0.0);

double FrontLeftSteer_MaxLower = frc::Shuffleboard::GetTab("Front Left")
.Add("SteerMax Lower", 0).GetEntry()->GetDouble(0.0);

double FrontRight_P = frc::Shuffleboard::GetTab("Front Right")
.Add("P", 0).GetEntry()->GetDouble(0.0);
double FrontRight_I = frc::Shuffleboard::GetTab("Front Right")
.Add("I", 0).GetEntry()->GetDouble(0.0);
double FrontRight_D = frc::Shuffleboard::GetTab("Front Right")
.Add("D", 0).GetEntry()->GetDouble(0.0);
double FrontRight_P_LL = frc::Shuffleboard::GetTab("Front Right")
.Add("P LL", 0).GetEntry()->GetDouble(0.0);
double FrontRight_P_UL = frc::Shuffleboard::GetTab("Front Right")
.Add("I UL", 0).GetEntry()->GetDouble(0.0);
double FrontRight_I_LL = frc::Shuffleboard::GetTab("Front Right")
.Add("I LL", 0).GetEntry()->GetDouble(0.0);
double FrontRight_I_UL = frc::Shuffleboard::GetTab("Front Right")
.Add("D UL", 0).GetEntry()->GetDouble(0.0);
double FrontRight_D_LL = frc::Shuffleboard::GetTab("Front Right")
.Add("D LL", 0).GetEntry()->GetDouble(0.0);
double FrontRight_MaxUpper = frc::Shuffleboard::GetTab("Front Right")
.Add("Max Upper", 0).GetEntry()->GetDouble(0.0);
double FrontRight_MaxLower = frc::Shuffleboard::GetTab("Front Right")
.Add("Max Lower", 0).GetEntry()->GetDouble(0.0);

double FrontRightSteer_P = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerP", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_I = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerI", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_D = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerD", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_P_LL = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerP LL", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_P_UL = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerI UL", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_I_LL = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerI LL", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_I_UL = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerD UL", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_D_LL = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerD LL", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_MaxUpper = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerMax Upper", 0).GetEntry()->GetDouble(0.0);
double FrontRightSteer_MaxLower = frc::Shuffleboard::GetTab("Front Right")
.Add("SteerMax Lower", 0).GetEntry()->GetDouble(0.0);

double BackLeft_P = frc::Shuffleboard::GetTab("Back Left")
.Add("P", 0).GetEntry()->GetDouble(0.0);
double BackLeft_I = frc::Shuffleboard::GetTab("Back Left")
.Add("I", 0).GetEntry()->GetDouble(0.0);
double BackLeft_D = frc::Shuffleboard::GetTab("Back Left")
.Add("D", 0).GetEntry()->GetDouble(0.0);
double BackLeft_P_LL = frc::Shuffleboard::GetTab("Back Left")
.Add("P LL", 0).GetEntry()->GetDouble(0.0);
double BackLeft_P_UL = frc::Shuffleboard::GetTab("Back Left")
.Add("I UL", 0).GetEntry()->GetDouble(0.0);
double BackLeftILL = frc::Shuffleboard::GetTab("Back Left")
.Add("I LL", 0).GetEntry()->GetDouble(0.0);
double BackLeftIUL = frc::Shuffleboard::GetTab("Back Left")
.Add("D UL", 0).GetEntry()->GetDouble(0.0);
double BackLeftDLL = frc::Shuffleboard::GetTab("Back Left")
.Add("D LL", 0).GetEntry()->GetDouble(0.0);
double BackLeftMaxUpper = frc::Shuffleboard::GetTab("Back Left")
.Add("Max Upper", 0).GetEntry()->GetDouble(0.0);
double BackLeftMaxLower = frc::Shuffleboard::GetTab("Back Left")
.Add("Max Lower", 0).GetEntry()->GetDouble(0.0);

double BackLeftSteer_P = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerP", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_I = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerI", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_D = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerD", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_PLL = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerP LL", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_PUL = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerI UL", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_I_LL = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerI LL", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_I_UL = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerD UL", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_D_LL = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerD LL", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_MaxUpper = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerMax Upper", 0).GetEntry()->GetDouble(0.0);
double BackLeftSteer_MaxLower = frc::Shuffleboard::GetTab("Back Left")
.Add("SteerMax Lower", 0).GetEntry()->GetDouble(0.0);

double BackRight_P = frc::Shuffleboard::GetTab("Back Right")
.Add("P", 0).GetEntry()->GetDouble(0.0);
double BackRight_I = frc::Shuffleboard::GetTab("Back Right")
.Add("I", 0).GetEntry()->GetDouble(0.0);
double BackRight_D = frc::Shuffleboard::GetTab("Back Right")
.Add("D", 0).GetEntry()->GetDouble(0.0);
double BackRight_P_LL = frc::Shuffleboard::GetTab("Back Right")
.Add("P LL", 0).GetEntry()->GetDouble(0.0);
double BackRight_P_UL = frc::Shuffleboard::GetTab("Back Right")
.Add("I UL", 0).GetEntry()->GetDouble(0.0);
double BackRight_I_LL = frc::Shuffleboard::GetTab("Back Right")
.Add("I LL", 0).GetEntry()->GetDouble(0.0);
double BackRight_I_UL = frc::Shuffleboard::GetTab("Back Right")
.Add("D UL", 0).GetEntry()->GetDouble(0.0);
double BackRight_D_LL = frc::Shuffleboard::GetTab("Back Right")
.Add("D LL", 0).GetEntry()->GetDouble(0.0);
double BackRight_MaxUpper = frc::Shuffleboard::GetTab("Back Right")
.Add("Max Upper", 0).GetEntry()->GetDouble(0.0);
double BackRight_MaxLower = frc::Shuffleboard::GetTab("Back Right")
.Add("Max Lower", 0).GetEntry()->GetDouble(0.0);

double BackRightSteer_P = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerP", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_I = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerI", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_D = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerD", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_P_LL = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerP LL", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_P_UL = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerI UL", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_I_LL = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerI LL", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_I_UL = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerD UL", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_D_LL = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerD LL", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_MaxUpper = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerMax Upper", 0).GetEntry()->GetDouble(0.0);
double BackRightSteer_MaxLower = frc::Shuffleboard::GetTab("Back Right")
.Add("SteerMax Lower", 0).GetEntry()->GetDouble(0.0);
/*
shuffleboard_FrontleftPID[E_PID_CalSz] = { 
                                            FrontLeft_P,
                                            FrontLeft_I,
                                            FrontLeft_D,
                                            FrontLeft_P_UL,
                                            FrontLeft_P_LL,
                                            FrontLeft_I_UL,
                                            FrontLeft_I_LL,
                                            FrontLeft_D_UL,
                                            FrontLeft_D_LL,
                                            FrontLeft_MaxUpper,
                                            FrontLeft_MaxLower,
};

shuffleboard_SteerFrontleftPID[E_PID_CalSz] = {        
                                            FrontLeftSteer_P,
                                            FrontLeftSteer_I,
                                            FrontLeftSteer_D,
                                            FrontLeftSteer_P_UL,
                                            FrontLeftSteer_P_LL,
                                            FrontLeftSteer_I_UL,
                                            FrontLeftSteer_I_LL,
                                            FrontLeftSteer_D_UL,
                                            FrontLeftSteer_D_LL,
                                            FrontLeftSteer_MaxUpper,
                                            FrontLeftSteer_MaxLower,
};                                                          
*/

}