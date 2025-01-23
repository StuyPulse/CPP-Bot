#pragma once 

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include "subsystems/SwerveDrive.h"

class SetXCommand
    : public frc2::CommandHelper<frc2::Command, SetXCommand> {
    
    public:
        explicit SetXCommand(SwerveDrive* subsystem);

        void Execute() override;
    
    private:
        SwerveDrive* m_drive;
};
