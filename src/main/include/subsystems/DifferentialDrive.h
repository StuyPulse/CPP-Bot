#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>

#include <Constants.h>

class DifferentialDrive : public frc2::SubsystemBase {
    public:
        DifferentialDrive();

        void ArcadeDrive(double xSpeed, double zRotation, bool squareInputs = true);
        void TankDrive(double leftSpeed, double rightSpeed, bool squareInputs = true);
    private:
        frc::PWMSparkMax m_left{DiffDrive::kLeftCanId};
        frc::PWMSparkMax m_right{DiffDrive::kRightCanId};
        frc::DifferentialDrive m_drive{m_left, m_right};
};