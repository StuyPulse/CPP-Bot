#include <subsystems/SwerveModule.h>

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"

using namespace rev::spark;

SwerveModule::SwerveModule(const int driveCANId, const int turnCANId, const double angleOffset)
    : driveMotor(driveCANId, SparkMax::MotorType::kBrushless),
      turnMotor(turnCANId, SparkMax::MotorType::kBrushless) {

    driveMotor.Configure(Configs::SwerveModule::DriveConfig(),
                         SparkBase::ResetMode::kResetSafeParameters,
                         SparkBase::PersistMode::kPersistParameters);
    
    turnMotor.Configure(Configs::SwerveModule::TurnConfig(),
                        SparkBase::ResetMode::kResetSafeParameters,
                        SparkMax::PersistMode::kPersistParameters);

    this->angleOffset = angleOffset;
    desiredState.angle = frc::Rotation2d(units::radian_t{turnEncoder.GetPosition()});
    driveEncoder.SetPosition(0);
}

frc::SwerveModuleState SwerveModule::GetState() const {
    return {units::meters_per_second_t{driveEncoder.GetVelocity()},
            units::radian_t{turnEncoder.GetPosition() - angleOffset}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
    return {units::meter_t{driveEncoder.GetPosition()},
            units::radian_t{turnEncoder.GetPosition() - angleOffset}};
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
    frc::SwerveModuleState newState{};
    newState.speed = desiredState.speed;
    newState.angle = desiredState.angle + frc::Rotation2d(units::radian_t{angleOffset});

    newState.Optimize(frc::Rotation2d(units::radian_t{turnEncoder.GetPosition()}));

    driveController.SetReference(
        (double)(newState.speed), SparkMax::ControlType::kVelocity);
    turnController.SetReference(
        newState.angle.Radians().value(), SparkMax::ControlType::kPosition);
    
    this->desiredState = desiredState;
}   

void SwerveModule::ResetEncoders() {
    driveEncoder.SetPosition(0);
}