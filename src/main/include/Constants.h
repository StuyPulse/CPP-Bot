// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

/**
 * CONSTANTS HAVE BEEN SET TO MATCH THOSE OF KITBOT-2025 (STUYPULSE 694)
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed = 3.0_rad_per_s;

constexpr double kDirectionSlewRate = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;  // percent per second (1 = 100%)

// Chassis configuration
constexpr units::meter_t kTrackWidth =
    {39.4_in};  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase =
    {34.3_in}; // Distance between centers of front and back wheels on robot

// Angular offsets of the modules relative to the chassis in radians
constexpr double kFrontLeftChassisAngularOffset {0.052734375};
constexpr double kFrontRightChassisAngularOffset {0.1318359375};
constexpr double kRearLeftChassisAngularOffset {0.33154296875};
constexpr double kRearRightChassisAngularOffset {0.192138671875 + 0.5};

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 14;
constexpr int kRearLeftDrivingCanId = 16;
constexpr int kFrontRightDrivingCanId = 12;
constexpr int kRearRightDrivingCanId = 10;

constexpr int kFrontLeftTurningCanId = 15;
constexpr int kRearLeftTurningCanId = 12;
constexpr int kFrontRightTurningCanId = 17;
constexpr int kRearRightTurningCanId = 11;
}  // namespace DriveConstants

namespace ModuleConstants {
// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps =
    5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter {4.0_in};
constexpr units::meter_t kWheelCircumference =
    kWheelDiameter * std::numbers::pi;
// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction =
    (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
constexpr double kDriveWheelFreeSpeedRps =
    (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) /
    kDrivingMotorReduction;

namespace Drive {
    constexpr double drivingFactor = kWheelCircumference.value() / kDrivingMotorReduction;
    constexpr double drivingVelocityFF = 1 / kDriveWheelFreeSpeedRps;
    constexpr double driveCurrentLimit = 40;

    constexpr double kP = 9.0;
    constexpr double kI = 0.0;
    constexpr double kD = 0.1;
}
namespace Turn {
    constexpr double turningFactor = 2 * std::numbers::pi;
    constexpr double turnCurrentLimit = 20;

    constexpr double kP = 9.0;
    constexpr double kI = 0.0;
    constexpr double kD = 0.2;
}
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 1.0_mps;
constexpr auto kMaxAcceleration = 10_mps_sq;
constexpr auto kMaxAngularSpeed = 3.0_rad_per_s;
constexpr auto kMaxAngularAcceleration = 25.0_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr double kDriveDeadband = 0.05;
}  // namespace OIConstants

namespace DiffDrive {
    constexpr int kLeftCanId = 0;
    constexpr int kRightCanId = 1;
}