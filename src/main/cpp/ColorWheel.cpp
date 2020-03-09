/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ColorWheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

ColorWheel::ColorWheel() 
{
    printf("Creating ColorWheel");
}

ColorWheel::~ColorWheel()
{
    printf("Deconstructing ColorWheel");
}

void ColorWheel::commies(double speed)
{
    commiessssss.Set(speed);
}

void ColorWheel::spin(double speed)
{
    spinnyyyyyyy.Set(speed);
}
