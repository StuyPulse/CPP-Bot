#pragma once

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>

class Mobility {
    public:
        static frc::Trajectory GetTrajectory(const frc::TrajectoryConfig& config);
};