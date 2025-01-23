#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/DifferentialDrive.h"

class DifferentialDriveCommand : public frc2::CommandHelper<frc2::Command, DifferentialDriveCommand> {
    public:
        explicit DifferentialDriveCommand(DifferentialDrive* drive, frc2::CommandXboxController* controller);

        void Execute() override;
    
    private:
        DifferentialDrive* m_drive;
        frc2::CommandXboxController* m_controller;
};