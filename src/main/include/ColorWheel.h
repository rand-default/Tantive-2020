/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <iostream>
#include <frc/PWMVictorSPX.h>

class ColorWheel {
 public:
  // Constructor
  ColorWheel();
  // Deconstructor
  ~ColorWheel();
  // Control winch for color wheel arm(russian roulette)
  void commies(double speed);
  // Control compression wheel to spin the roulette wheel
  void spin(double speed);
 private:
  // Create talon objects
  frc::PWMVictorSPX commiessssss{5};
  frc::PWMVictorSPX spinnyyyyyyy{6};

};
