#include "subsystems/DifferentialDrive.h"

DifferentialDrive::DifferentialDrive() {
    m_right.SetInverted(true);
}

void DifferentialDrive::ArcadeDrive(double speed, double rotation, bool squareInputs) {
    m_drive.ArcadeDrive(speed, rotation, squareInputs);
}

void DifferentialDrive::TankDrive(double leftSpeed, double rightSpeed, bool squareInputs) {
    m_drive.TankDrive(leftSpeed, rightSpeed, squareInputs);
}