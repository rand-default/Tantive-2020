#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/color.h>

  cs::UsbCamera camera1;
  cs::UsbCamera camera2;
  cs::VideoSink server1;

void Robot::RobotInit() 
{
  m_colorMatcher.AddColorMatch(kBlueTarget);
  m_colorMatcher.AddColorMatch(kGreenTarget);
  m_colorMatcher.AddColorMatch(kRedTarget);
  m_colorMatcher.AddColorMatch(kYellowTarget);

  camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  server1 = frc::CameraServer::GetInstance()->GetServer();
  camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  camera2.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

  // std::thread visionThread(VisionThread);
  // VisionThread.detach();
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
  if(colorString=="Blue")
  {
    std::cout << "Sensed color:   Red\n";
  }
  else if(colorString=="Yellow")
  {
    std::cout << "Sensed color:   Green\n";
  }
  else if(colorString=="Red")
  {
    std::cout << "Sensed color:   Blue\n";
  }
  else if(colorString=="Green")
  {
    std::cout << "Sensed color:   Yellow\n";
  }
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
  // Code for joystick, only the drivetrain(driver)
  if (joystick.trigger())
  {
    chassis.LRSpeed(joystick.xisis()*joystick.xisis()*joystick.xisis()-joystick.yisis()*joystick.yisis()*joystick.yisis(), joystick.yisis()*joystick.yisis()*joystick.yisis()+joystick.xisis()*joystick.xisis()*joystick.xisis());
  }

  else if (joystick.thumbb())
  {
    chassis.LRSpeed(joystick.yisis()*joystick.yisis()*joystick.yisis(), 0);
  }
  
  // else if (joystick.button5())
  // {
  //   chassis.LRSpeed(0.7, 0.7);
  // }

  // else if (joystick.button3())
  // {
  //   chassis.LRSpeed(0.3, 0.3);
  // }

  else
  {
  //chassis.LRSpeed(abs(joystick.xisis())*joystick.xisis() - abs(joystick.yisis())*joystick.yisis(), abs(joystick.xisis())*joystick.xisis() + abs(joystick.yisis())*joystick.yisis());
  chassis.LRSpeed(joystick.yisis()+joystick.xisis(), joystick.xisis()-joystick.yisis());
  }

  // Code for the Driver Station, all other motors(co-driver)
  
  // Intake winch raise/lower
  if(station.red1())
  {
    intake.winch(0.5);
  }
  else if(station.red2())
  {
    intake.winch(-0.5);
  }
  else
  {
    intake.winch(0.0);
  }

  // Transfer forward/reverse
  if(station.green1())
  {
    intake.trans(-0.3, 0.3);
  }
  else if(station.green2())
  {
    intake.trans(0.3, -0.3);
  }
  else
  {
    intake.trans(0.0,0.0);
  }

  // Climber up/down
  if(station.blue1())
  {
    climb.up(0.3);
  }
  else if(station.blue2())
  {
    climb.up(-0.7);
  }
  else
  {
    climb.up(0.0);
  }
  
  // Camera selection
  // Front(intake)
  if(station.yellow1())
  {
    printf("Setting camera 1\n");
    server1.SetSource(camera1);
  }
  // Back(output balls)
  else if(station.yellow2())
  {
    printf("Setting camera 2\n");
    server1.SetSource(camera2);
  }

  // Roulette wheel arm code
  // Winch raise/lower for arm
  if(joystick.button3())
  {
    printf("Lowering roulette arm\n");
    roulette.commies(-0.3);
  }
  else if(joystick.button5())
  {
    printf("Raising roulette arm\n");
    roulette.commies(0.3);
  }
  else
  {
    roulette.commies(0.0);
  }
  // Spin the compression wheel
  // Crank it(;
  if(joystick.button6())
  {
    roulette.spin(0.7);
  }
  // Slow it down homie
  else if(joystick.button4())
  {
    roulette.spin(0.2);
  }
  else
  {
    roulette.spin(0.0);
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
