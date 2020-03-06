/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "IntakeTransfer.h"
#include <frc/smartdashboard/SmartDashboard.h>

IntakeTransfer::IntakeTransfer()
{
    printf("Creating IntakeTransfer");
}

IntakeTransfer::~IntakeTransfer()
{
    printf("Deconstructing IntakeTransfer");
}

void IntakeTransfer::winch(double speed)
{
    WinchMotor.Set(speed);
}

void IntakeTransfer::top(double speed)
{
    upper.Set(speed);
}

void IntakeTransfer::bottom(double speed)
{
    lower.Set(speed);
}

void IntakeTransfer::trans(double UpperSpeed, double LowerSpeed)
{
    upper.Set(UpperSpeed);
    lower.Set(LowerSpeed);
}
