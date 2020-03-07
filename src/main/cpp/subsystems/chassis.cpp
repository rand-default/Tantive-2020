#include "subsystems/chassis.h"
#include <frc/smartdashboard/SmartDashboard.h>

Chassis::Chassis()
{
    printf("Creating Chassis...");
}

Chassis::~Chassis()
{
    printf("Deconstructing Chassis");
}

void Chassis::left(double speed)
{
    LeftMaster.Set(-speed);
    LeftSlave.Set(-speed);
}

void Chassis::right(double speed)
{
    RightMaster.Set(speed);
    RightSlave.Set(speed);
}

void Chassis::LRSpeed(double LSpeed, double RSpeed)
{
    LeftMaster.Set(LSpeed);
    LeftSlave.Set(LSpeed);
    RightMaster.Set(RSpeed);
    RightSlave.Set(RSpeed);
}

void Chassis::neogeo(double speed)
{
    Brushless.Set(speed);
    std::cout << "Position : " << encoder.GetPosition() << std::endl;
    // frc::SmartDashboard::PutNumber("ProcessVariable", encoder.GetPosition());
}