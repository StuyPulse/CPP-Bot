#include <subsystems/SwerveModule.h>

using namespace rev::spark;

SwerveModule::SwerveModule(const int driveCANId, const int turnCANId, const double angleOffset)
    : driveMotor(driveCANId, SparkMax::MotorType::kBrushless),
      turnMotor(turnCANId, SparkMax::MotorType::kBrushless) {

    // Configure motors

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