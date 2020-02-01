
#pragma once

#include <frc/TimedRobot.h>
#include "subsystems/chassis.h"
#include "subsystems/chassis.h"
#include "subsystems/OurJoystick.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
    Chassis chassis;
    OurJoystick joystick;
    
};