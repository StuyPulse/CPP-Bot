#include "commands/auton/DifferentialMobility.h"

DifferentialMobility::DifferentialMobility(DifferentialDrive* drive)
    : m_drive{drive} {
    AddRequirements(drive);
}

void DifferentialMobility::Initialize() {
    m_timer.Reset();
    m_timer.Start();
}

void DifferentialMobility::Execute() {
    if (m_timer.Get() < 2_s) {
        m_drive->ArcadeDrive(0.5, 0.0, false);
    } else {
        m_drive->ArcadeDrive(0.0, 0.0, false);
    }

    // if (m_timer.Get() < 2_s) {
    //     m_drive->TankDrive(0.5, 0.5, false);
    // } else {
    //     m_drive->TankDrive(0.0, 0.0, false);
    // }
}

void DifferentialMobility::End(bool interrupted) {
  m_drive->ArcadeDrive(0.0, 0.0, false);
}