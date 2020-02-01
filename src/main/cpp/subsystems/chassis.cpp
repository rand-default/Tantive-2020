#include "subsystems/chassis.h"

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
    LeftMaster.Set(speed);
    LeftSlave.Set(speed);
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
    // RightMaster.Set(RSpeed);
    // RightSlave.Set(RSpeed);
}

void Chassis::neogeo(double speed)
{
    // Brushless.SetSpeed(0.3);
    Brushless.SetPosition(999);
 
    // std::cout << "RAW : " << Brushless.GetRaw() << std::endl;
    // std::cout << "Speed : " << Brushless.GetSpeed() << std::endl;
    // std::cout << "Error : " << Brushless.GetError() << std::endl;
    std::cout << "Position : " << Brushless.GetPosition() << std::endl;
}