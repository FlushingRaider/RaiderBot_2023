#pragma once

#include <string>

#include "PIDConfig.hpp"

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>

class ShooterConfig
{
    public:
        PIDConfig rightShooterPIDConfig;
        PIDConfig leftShooterPIDConfig;
        double TopDesiredSpeed;
        double BottomDesiredSpeed;

        ShooterConfig();

        ShooterConfig(PIDConfig rightShooterPIDConfig, PIDConfig leftShooterPIDConfig);

        void Debug(std::string TabName);
    private:
        bool init;
        nt::NetworkTableEntry nt_topDesiredSpeed, nt_bottomDesiredSpeed;
};

ShooterConfig::ShooterConfig(){}

ShooterConfig::ShooterConfig(PIDConfig rightShooterPIDConfig, PIDConfig leftShooterPIDConfig)
{
    rightShooterPIDConfig = rightShooterPIDConfig;
    leftShooterPIDConfig = leftShooterPIDConfig;
}

void ShooterConfig::Debug(std::string TabName)
{
    static auto& Tab = frc::Shuffleboard::GetTab(TabName);

    rightShooterPIDConfig.Debug("(Top PID)"+TabName);
    leftShooterPIDConfig.Debug("(Bottom PID)"+TabName);

    if(!init)
    {
        nt_topDesiredSpeed = Tab.Add("Top Desired Speed", TopDesiredSpeed).WithPosition(0,0).GetEntry();
        nt_bottomDesiredSpeed = Tab.Add("Bottom Desired Speed", BottomDesiredSpeed).WithPosition(0,1).GetEntry();

        init = true;
    }

    TopDesiredSpeed = nt_topDesiredSpeed.GetDouble(TopDesiredSpeed);
    BottomDesiredSpeed = nt_bottomDesiredSpeed.GetDouble(BottomDesiredSpeed);

}
