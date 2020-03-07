
#pragma once

#include <frc/TimedRobot.h>
#include "subsystems/chassis.h"
#include "subsystems/OurJoystick.h"
#include "DriverStation.h"
#include "IntakeTransfer.h"
#include "Climber.h"
#include "ColorSense.h"
#include <C:\Users\Admin\.gradle\caches\transforms-2\files-2.1\668ed1c1966c97fb577eede7f2896be1\cameraserver-cpp-2020.2.2-headers\cameraserver\CameraServer.h>
#include <C:\Users\Admin\.gradle\caches\transforms-2\files-2.1\eed3931469ba7efc2e953f8f3443a673\opencv-cpp-3.4.7-2-headers\opencv2\core\core.hpp>
#include <C:\Users\Admin\.gradle\caches\transforms-2\files-2.1\eed3931469ba7efc2e953f8f3443a673\opencv-cpp-3.4.7-2-headers\opencv2\imgproc\imgproc.hpp>
#include <C:\Users\Admin\.gradle\caches\transforms-2\files-2.1\668ed1c1966c97fb577eede7f2896be1\cameraserver-cpp-2020.2.2-headers\cameraserver\CameraServerShared.h>
#include <C:\Users\Admin\.gradle\caches\transforms-2\files-2.1\b97702da5f4ce7738418ef1913e15e56\cscore-cpp-2020.2.2-headers\cscore.h>

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
    static void VisionThread()
    {
        // cs::UsbCamera camera = CameraServer::GetInstance()-StartAutomaticCapture();
        // camera.SetResolution(640, 480);
        // cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
        // cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
        // cv::Mat source;
        // cv::Mat output;
        // while(true)
        // {
        //     cvSink.GrabFrame(source);
        //     cvtColor(source, output, cv::COLOR_BGR2GRAY);
        //     ourputStreamStd.PutFrame(output);
        // }
    }
    Chassis chassis;
    OurJoystick joystick;
    DriverStation station;
    IntakeTransfer intake;
    Climber climb;
};