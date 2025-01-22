#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "SwerveModule.h"
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

class SwerveDrive : public frc2::SubsystemBase {
    public:
        SwerveDrive();

        void Periodic() override;

        void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                   units::radians_per_second_t rot, bool fieldRelative);
        
        void SetX();

        void ResetEncoders();

        void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

        units::degree_t GetHeading() const;

        void ZeroHeading();

        double GetTurnRate();

        frc::Pose2d GetPose();

        void ResetOdometry(frc::Pose2d pose);

        frc::SwerveDriveKinematics<4> kDriveKinematics {
            frc::Translation2d{DriveConstants::kWheelBase / 2,
                               DriveConstants::kTrackWidth / 2},
            frc::Translation2d{DriveConstants::kWheelBase / 2,
                               -DriveConstants::kTrackWidth / 2},
            frc::Translation2d{-DriveConstants::kWheelBase / 2,
                               DriveConstants::kTrackWidth / 2},
            frc::Translation2d{-DriveConstants::kWheelBase / 2,
                               -DriveConstants::kTrackWidth / 2}
        };

    private:
        SwerveModule frontRight;
        SwerveModule frontLeft;
        SwerveModule rearRight;
        SwerveModule rearLeft;

        frc::ADIS16470_IMU gyro;

        frc::SwerveDriveOdometry<4> odometry;
};