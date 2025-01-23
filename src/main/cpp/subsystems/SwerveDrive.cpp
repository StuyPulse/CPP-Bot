#include "subsystems/SwerveDrive.h"

#include "Constants.h"

using namespace DriveConstants;

SwerveDrive::SwerveDrive() 
    : frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId, kFrontLeftChassisAngularOffset},
      rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId, kRearLeftChassisAngularOffset},
      frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId, kFrontRightChassisAngularOffset},
      rearRight{kRearRightDrivingCanId, kRearRightTurningCanId, kRearRightChassisAngularOffset},
      odometry{kDriveKinematics,
               frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
               {frontRight.GetPosition(), frontLeft.GetPosition(), rearRight.GetPosition(), rearLeft.GetPosition()},
               frc::Pose2d{}} {}

void SwerveDrive::Periodic() {
    odometry.Update(
        frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
        {frontLeft.GetPosition(), rearLeft.GetPosition(), frontRight.GetPosition(), rearRight.GetPosition()}
    );
}

void SwerveDrive::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                        units::radians_per_second_t rot, bool fieldRelative) {
    units::meters_per_second_t xSpeedDelivered = xSpeed.value() * kMaxSpeed;
    units::meters_per_second_t ySpeedDelivered = ySpeed.value() * kMaxSpeed;
    units::radians_per_second_t rotDelivered = rot.value() * kMaxAngularSpeed;

    auto states = kDriveKinematics.ToSwerveModuleStates(
        fieldRelative
            ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}))
            : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered}
    );

    kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

    auto [fl, fr, rl, rr] = states;

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    rearLeft.SetDesiredState(rl);
    rearRight.SetDesiredState(rr);
}

void SwerveDrive::SetX() {
    frontLeft.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
    frontRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    rearLeft.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    rearRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void SwerveDrive::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMaxSpeed);

    frontLeft.SetDesiredState(desiredStates[0]);
    frontRight.SetDesiredState(desiredStates[1]);
    rearLeft.SetDesiredState(desiredStates[2]);
    rearRight.SetDesiredState(desiredStates[3]);
}

void SwerveDrive::ResetEncoders() {
    frontLeft.ResetEncoders();
    frontRight.ResetEncoders();
    rearLeft.ResetEncoders();
    rearRight.ResetEncoders();
}

units::degree_t SwerveDrive::GetHeading() const {
    return frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}).Degrees();
}

void SwerveDrive::ZeroHeading() {
    gyro.Reset();
}

double SwerveDrive::GetTurnRate() {
    return -gyro.GetRate(frc::ADIS16470_IMU::IMUAxis::kZ).value();
}

frc::Pose2d SwerveDrive::GetPose() {
    return odometry.GetPose();
}

void SwerveDrive::ResetOdometry(frc::Pose2d pose) {
    odometry.ResetPosition(
        GetHeading(),
        {frontLeft.GetPosition(), frontRight.GetPosition(), rearLeft.GetPosition(), rearRight.GetPosition()},
        pose
    );
}