#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <rev/SparkMax.h>

using namespace rev::spark;

class SwerveModule {
    public:
        SwerveModule(int driveCANId, int turnCANId, double angleOffset);
        
        frc::SwerveModuleState GetState() const;
        
        frc::SwerveModulePosition GetPosition() const;

        void SetDesiredState(const frc::SwerveModuleState& state);

        void ResetEncoders();
    
    private:
        SparkMax driveMotor;
        SparkMax turnMotor;

        SparkRelativeEncoder driveEncoder = driveMotor.GetEncoder();
        SparkAbsoluteEncoder turnEncoder = turnMotor.GetAbsoluteEncoder();

        SparkClosedLoopController driveController = driveMotor.GetClosedLoopController();
        SparkClosedLoopController turnController = turnMotor.GetClosedLoopController();

        double angleOffset = 0.0;
        frc::SwerveModuleState desiredState = {units::meters_per_second_t{0.0}, frc::Rotation2d()};
};