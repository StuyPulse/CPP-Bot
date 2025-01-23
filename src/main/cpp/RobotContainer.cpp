// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>

#include "commands/Autos.h"

RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here
	m_drive.SetDefaultCommand(DriveCommand{&m_drive, &m_driverController});
	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	m_driverController.RightBumper()
		.WhileTrue(&m_setXCommand);
} 

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	// // An example command will be run in autonomous
	// return autos::ExampleAuto(&m_subsystem);
}
