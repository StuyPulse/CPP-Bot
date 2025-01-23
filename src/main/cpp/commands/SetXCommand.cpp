#include "commands/SetXCommand.h"

SetXCommand::SetXCommand(SwerveDrive* driveSubsystem)
    : m_drive{driveSubsystem} {
    AddRequirements(m_drive);
}

void SetXCommand::Execute() {
    m_drive->SetX();
}