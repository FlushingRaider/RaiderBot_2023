//Auton Path loader
//Wes: 2/19/23

#pragma once

#include <fstream>
#include <vector>
#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <string>

#include "json.hpp"

struct AutonPath
{
  std::vector<double> time;
  std::vector<double> rot;
  std::vector<double> x;
  std::vector<double> y;
  std::string Name;
};

AutonPath PathLoader(std::string PathName);

