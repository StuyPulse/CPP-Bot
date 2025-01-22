#include "subsystems/SwerveDrive.h"

#include "Constants.h"

using namespace DriveConstants;

SwerveDrive::SwerveDrive() 
    : frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId, kFrontRightChassisAngularOffset},
      frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId, kFrontLeftChassisAngularOffset},
      rearRight{kRearRightDrivingCanId, kRearRightTurningCanId, kRearRightChassisAngularOffset},
      rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId, kRearLeftChassisAngularOffset},
      odometry{kDriveKinematics,
               frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
               {frontRight.GetPosition(), frontLeft.GetPosition(), rearRight.GetPosition(), rearLeft.GetPosition()},
               frc::Pose2d{}} {};

void SwerveDrive::Periodic() {
    odometry.Update(
        frc::Rotation2d(units::radian_t{gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kZ)}),
        {frontRight.GetPosition(), frontLeft.GetPosition(), rearRight.GetPosition(), rearLeft.GetPosition()}
    );
}

void SwerveDrive::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                        units::radians_per_second_t rot, bool fieldRelative) {

}

void SwerveDrive::SetX() {
    frontRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
    frontLeft.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
    rearRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
    rearLeft.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
}

void SwerveDrive::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
    kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMaxSpeed);

    frontRight.SetDesiredState(desiredStates[0]);
    frontLeft.SetDesiredState(desiredStates[1]);
    rearRight.SetDesiredState(desiredStates[2]);
    rearLeft.SetDesiredState(desiredStates[3]);
}

void SwerveDrive::ResetEncoders() {
    frontRight.ResetEncoders();
    frontLeft.ResetEncoders();
    rearRight.ResetEncoders();
    rearLeft.ResetEncoders();
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
        {frontRight.GetPosition(), frontLeft.GetPosition(), rearRight.GetPosition(), rearLeft.GetPosition()},
        pose
    );
}