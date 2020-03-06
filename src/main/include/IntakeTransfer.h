/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <iostream>
#include <frc/Spark.h>

class IntakeTransfer {
 public:
  // Constructor
  IntakeTransfer();
  // Deconstructor
  ~IntakeTransfer();
  // Control intake winch
  void winch(double speed);
  // Control top half
  void top(double UpperSpeed);
  // Control bottom half
  void bottom(double LowerSpeed);
  // Control top and bottom
  void trans(double UpperSpeed, double LowerSpeed);
 private:
  // Create Spark Objects
  frc::Spark WinchMotor{2};
  frc::Spark upper{3};
  frc::Spark lower{4};
};
