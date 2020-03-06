/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/Joystick.h>

#pragma once

class DriverStation {
 public:
  bool red1();
  bool red2();
  bool green1();
  bool green2();
  bool blue1();
  bool blue2();
  bool yellow1();
  bool yellow2();
  bool white();

 private:
  frc::Joystick DriverStation{1};
};
