#include "commands/DifferentialDriveCommand.h"

DifferentialDriveCommand::DifferentialDriveCommand(DifferentialDrive* drive, frc2::CommandXboxController* controller)
    : m_drive{drive}, m_controller{controller} {
    AddRequirements(drive);
}

void DifferentialDriveCommand::Execute() {
    m_drive->ArcadeDrive(-m_controller->GetLeftY(), m_controller->GetRightX());
    // m_drive->TankDrive(-m_controller->GetLeftY(), -m_controller->GetRightY());
}