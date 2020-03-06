/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <iostream>
#include <frc/spark.h>

class Climber {
 public:
 // Constructor
 Climber();
 // Deconstructor
 ~Climber();
 // Control Climber
 void up(double speed);

 private:
  // Create Spark Objects
  frc::Spark ClimbMaster{0};
  frc::Spark ClimbSlave{1};
};
