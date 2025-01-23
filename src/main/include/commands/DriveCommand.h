#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/SwerveDrive.h"

class DriveCommand
    : public frc2::CommandHelper<frc2::Command, DriveCommand> {

    public:
        explicit DriveCommand(SwerveDrive* subsystem, frc2::CommandXboxController* controller);

        void Execute() override;
    
    private:
        SwerveDrive* m_drive;
        frc2::CommandXboxController* m_controller;
};