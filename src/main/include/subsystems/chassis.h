#pragma once

#include <iostream>
#include <frc/Spark.h>
#include <sparkmax/rev/CANSparkMax.h>

class Chassis
{
public:
  // Constructor
  Chassis(); 
  // Deconstructor
  ~Chassis();
  //Control brushless speed
  void neogeo(double speed);
  // Control Left Motor Speed
  void left(double speed);
  // Control Right Motor Speed
  void right(double speed);
  // Control both sides in one function
  void LRSpeed(double LSpeed, double RSpeed);

private:
  // Create Spark Objects
  frc::Spark LeftMaster{9};
  frc::Spark LeftSlave{8};
  frc::Spark RightMaster{6};
  frc::Spark RightSlave{7};
  // rev::SparkMax Brushless{5};

  rev::CANSparkMax Brushless{1};
  
};
