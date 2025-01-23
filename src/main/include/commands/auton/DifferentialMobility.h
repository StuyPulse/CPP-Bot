#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc/Timer.h>
#include "subsystems/DifferentialDrive.h"

class DifferentialMobility : public frc2::CommandHelper<frc2::Command, DifferentialMobility> {
    public:
        explicit DifferentialMobility(DifferentialDrive* drive);
    
        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;

    private:
        DifferentialDrive* m_drive;
        frc::Timer m_timer;
};