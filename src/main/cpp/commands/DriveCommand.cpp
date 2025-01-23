#include "commands/DriveCommand.h"
#include <frc/MathUtil.h>

DriveCommand::DriveCommand(SwerveDrive* driveSubsystem, frc2::CommandXboxController* controller)
    : m_drive{driveSubsystem}, m_controller{controller} {
        AddRequirements(m_drive);
}

void DriveCommand::Execute() {
    m_drive->Drive(
        -units::meters_per_second_t{frc::ApplyDeadband(
            m_controller->GetLeftY(), OIConstants::kDriveDeadband)},
        -units::meters_per_second_t{frc::ApplyDeadband(
            m_controller->GetLeftX(), OIConstants::kDriveDeadband)},
        -units::radians_per_second_t{frc::ApplyDeadband(
            m_controller->GetRightX(), OIConstants::kDriveDeadband)},
        true
    );
}