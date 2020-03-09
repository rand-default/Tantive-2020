#pragma once
#include <frc/Joystick.h>

class OurJoystick
{
public:
  double xisis();
  double yisis();
  double twistyy();
  bool trigger();
  bool thumbb();
  bool button5();
  bool button3();
  bool button4();
  bool button6();

private:
  frc::Joystick stick{0};
};
