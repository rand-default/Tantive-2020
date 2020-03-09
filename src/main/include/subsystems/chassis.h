#pragma once

#include <iostream>
#include <frc/Spark.h>
// #include <sparkmax/rev/CANSparkMax.h>
#include <rev/CANSparkMax.h>
#include <frc/PWMVictorSPX.h>

class Chassis
{
public:
  // Constructor
  Chassis(); 
  // Deconstructor
  ~Chassis();
  //Control brushless speed
  void neogeo(double speed);
  // // Control Left Motor Speed
  // void left(double speed);
  // // Control Right Motor Speed
  // void right(double speed);
  // Control both sides in one function
  void LRSpeed(double LSpeed, double RSpeed);
private:
  // Create Spark Objects
  frc::Spark Left{9};
  frc::Spark Right{8};

  // THE SACRED COODE
  rev::CANSparkMax Brushless{1, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANEncoder encoder = Brushless.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 4096);
};