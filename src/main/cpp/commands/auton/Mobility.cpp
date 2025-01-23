#include "commands/auton/Mobility.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>

using namespace frc;

Trajectory Mobility::GetTrajectory(const TrajectoryConfig& config) {
    return TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{0_m, 0_m, 0_deg},
        {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
        frc::Pose2d{3_m, 0_m, 0_deg},
        config
    );
}