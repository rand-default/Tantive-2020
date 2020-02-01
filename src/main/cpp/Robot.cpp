#include "Robot.h"

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic()
{
  chassis.left(0.5);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
  // if (joystick.trigger())
  // {
  //   chassis.left(abs(joystick.yisis())*joystick.yisis());
  // }

  // if (joystick.thumbb())
  // {
  //   chassis.left(joystick.yisis()/2);
  // }
  
  // if (joystick.button5())
  // {
  //   chassis.left(1);
  // }

  // if (joystick.button3())
  // {
  //   chassis.left(-1);
  // }

  // else
  // {
  //   // chassis.left(joystick.yisis()*joystick.yisis()*joystick.yisis());
  // }
}

void Robot::TestPeriodic()
{
  chassis.neogeo(joystick.yisis()*joystick.yisis()*joystick.yisis());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
