#pragma once

#include <Constants.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

namespace Configs {
    class SwerveModule {
        public:
            static SparkMaxConfig& DriveConfig() {
                static SparkMaxConfig drivingConfig{};

                drivingConfig.SetIdleMode(SparkBaseConfig::kBrake)
                    .SmartCurrentLimit(ModuleConstants::Drive::driveCurrentLimit);
                drivingConfig.encoder
                    .PositionConversionFactor(ModuleConstants::Drive::drivingFactor)
                    .VelocityConversionFactor(ModuleConstants::Drive::drivingFactor / 60.0);
                drivingConfig.closedLoop
                    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                    .Pid(ModuleConstants::Drive::kP, ModuleConstants::Drive::kI, ModuleConstants::Drive::kD)
                    .VelocityFF(ModuleConstants::Drive::drivingVelocityFF)
                    .OutputRange(-1, 1);

                return drivingConfig;
            }

            static SparkMaxConfig& TurnConfig() {
                static SparkMaxConfig turningConfig{};

                turningConfig.SetIdleMode(SparkBaseConfig::IdleMode::kBrake)
                    .SmartCurrentLimit(ModuleConstants::Turn::turnCurrentLimit);
                turningConfig.absoluteEncoder
                    .Inverted(true)
                    .PositionConversionFactor(ModuleConstants::Turn::turningFactor)
                    .VelocityConversionFactor(ModuleConstants::Turn::turningFactor / 60.0);
                turningConfig.closedLoop
                    .SetFeedbackSensor(ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
                    .Pid(ModuleConstants::Turn::kP, ModuleConstants::Turn::kI, ModuleConstants::Turn::kD)
                    .OutputRange(-1, 1)
                    .PositionWrappingEnabled(true)
                    .PositionWrappingInputRange(0, ModuleConstants::Turn::turningFactor);

                return turningConfig;
            }
    };
}