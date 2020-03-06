/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Climber.h"
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber()
{
    printf("Creating Climber");
}

Climber::~Climber()
{
    printf("Deconstructing Climber");
}

void Climber::up(double speed)
{
    ClimbMaster.Set(speed);
    ClimbSlave.Set(speed);
}