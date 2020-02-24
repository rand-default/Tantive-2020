
#pragma once

#include <frc/TimedRobot.h>
#include "subsystems/chassis.h"
#include "subsystems/OurJoystick.h"
#include "ColorSense.h"

class Robot : public frc::TimedRobot {
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
    
    rev::ColorSensorV3 m_colorSensor{i2cPort};
    rev::ColorMatch m_colorMatcher;

    static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
    static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
    static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
    static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);

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