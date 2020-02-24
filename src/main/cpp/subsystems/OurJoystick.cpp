// Some credit to frc
#include "subsystems/OurJoystick.h"
#include "subsystems/chassis.h"

double OurJoystick::xisis()
{
    return stick.GetRawAxis(0);
}

double OurJoystick::yisis()
{
    return stick.GetRawAxis(1);
}

double OurJoystick::twistyy()
{
    return stick.GetRawAxis(2);
}

bool OurJoystick::trigger()
{
    return stick.GetRawButton(1);
}

bool OurJoystick::thumbb()
{
    return stick.GetRawButton(2);
}

bool OurJoystick::button3()
{
    return stick.GetRawButton(3);
}

bool OurJoystick::button5()
{
    return stick.GetRawButton(5);
}