#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/color.h>
#include <C:\Users\Admin\.gradle\caches\transforms-2\files-2.1\668ed1c1966c97fb577eede7f2896be1\cameraserver-cpp-2020.2.2-headers\cameraserver\CameraServer.h>

void Robot::RobotInit() 
{
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);

frc::CameraServer::GetInstance()->StartAutomaticCapture();
}

void Robot::RobotPeriodic() 
{
  frc::Color detectedColor = m_colorSensor.GetColor();

  double IR = m_colorSensor.GetIR();

  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("IR", IR);

  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

  uint32_t proximity = m_colorSensor.GetProximity();

  if (matchedColor == kBlueTarget) {
    colorString = "Blue";
  } else if (matchedColor == kRedTarget) {
    colorString = "Red";
  } else if (matchedColor == kGreenTarget) {
    colorString = "Green";
  } else if (matchedColor == kYellowTarget) {
    colorString = "Yellow";
  } else {
    colorString = "Unknown";
  }

  frc::SmartDashboard::PutNumber("Proximity", proximity);
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  frc::SmartDashboard::PutNumber("Confidence", confidence);
  frc::SmartDashboard::PutString("Detected Color", colorString);
  std::cout << "Detected Color: " << colorString << std::endl;
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic()
{
  chassis.LRSpeed(0.5, 0.5);
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{  
  if (joystick.trigger())
  {
    chassis.LRSpeed(joystick.xisis()*joystick.xisis()*joystick.xisis()-joystick.yisis()*joystick.yisis()*joystick.yisis(), joystick.yisis()*joystick.yisis()*joystick.yisis()+joystick.xisis()*joystick.xisis()*joystick.xisis());
  }

  else if (joystick.thumbb())
  {
    chassis.LRSpeed(joystick.yisis()*joystick.yisis()*joystick.yisis(), 0);
  }
  
  else if (joystick.button5())
  {
    chassis.LRSpeed(0.7, 0.7);
  }

  else if (joystick.button3())
  {
    chassis.LRSpeed(0.3, 0.3);
  }

  else
  {
  //chassis.LRSpeed(abs(joystick.xisis())*joystick.xisis() - abs(joystick.yisis())*joystick.yisis(), abs(joystick.xisis())*joystick.xisis() + abs(joystick.yisis())*joystick.yisis());
  chassis.LRSpeed(joystick.xisis()-joystick.yisis(), joystick.xisis()+joystick.yisis());
  }
}

void Robot::TestPeriodic()
{
  chassis.neogeo(abs(joystick.yisis())*joystick.yisis());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
