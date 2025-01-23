// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h"
#include "commands/SetXCommand.h"
#include "commands/DriveCommand.h"
#include "commands/auton/Mobility.h"

#include "subsystems/DifferentialDrive.h"
#include "commands/auton/DifferentialMobility.h"
#include "commands/DifferentialDriveCommand.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  SwerveDrive m_drive;

  SetXCommand m_setXCommand{&m_drive};

  DriveCommand m_driveCommand{&m_drive, &m_driverController};

  DifferentialDrive m_differentialDrive;
  DifferentialDriveCommand m_differentialDriveCommand{&m_differentialDrive, &m_driverController};
  DifferentialMobility m_differentialMobility{&m_differentialDrive};

  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureBindings();
};
