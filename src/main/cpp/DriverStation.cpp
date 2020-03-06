/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "DriverStation.h"
#include "subsystems/chassis.h"

bool DriverStation::red1()
{
    return DriverStation.GetRawButton(1);
}

bool DriverStation::red2()
{
    return DriverStation.GetRawButton(2);
}

bool DriverStation::green1()
{
    return DriverStation.GetRawButton(3);
}

bool DriverStation::green2()
{
    return DriverStation.GetRawButton(4);
}

bool DriverStation::blue1()
{
    return DriverStation.GetRawButton(5);
}

bool DriverStation::blue2()
{
    return DriverStation.GetRawButton(6);
}

bool DriverStation::yellow1()
{
    return DriverStation.GetRawButton(7);
}

bool DriverStation::yellow2()
{
    return DriverStation.GetRawButton(8);
}

bool DriverStation::white()
{
    return DriverStation.GetRawButton(9);
}