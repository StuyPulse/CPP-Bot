// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>	
#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here
	m_drive.SetDefaultCommand(std::move(m_driveCommand));
	// m_differentialDrive.SetDefaultCommand(std::move(m_differentialDriveCommand));
	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	m_driverController.RightBumper()
		.WhileTrue(&m_setXCommand);
} 

frc2::Command* RobotContainer::GetAutonomousCommand() {

	frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
	config.SetKinematics(m_drive.kDriveKinematics);

	auto trajectory = Mobility::GetTrajectory(config);

	frc::ProfiledPIDController<units::radians> thetaController {
		AutoConstants::kPThetaController, 0, 0,
		AutoConstants::kThetaControllerConstraints
	};

	thetaController.EnableContinuousInput(
		units::radian_t{-std::numbers::pi},
		units::radian_t{std::numbers::pi}
	);

	frc2::SwerveControllerCommand<4> swerveControllerCommand(
		trajectory,
		[this]() { return m_drive.GetPose(); },
		m_drive.kDriveKinematics,
		frc::PIDController{AutoConstants::kPXController, 0, 0},
		frc::PIDController{AutoConstants::kPYController, 0, 0},
		thetaController,
		[this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
		{&m_drive}
	);

	m_drive.ResetOdometry(trajectory.InitialPose());

	return new frc2::SequentialCommandGroup(
		std::move(swerveControllerCommand),
		frc2::InstantCommand(
			[this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {}));

	// return &m_differentialMobility;
}
