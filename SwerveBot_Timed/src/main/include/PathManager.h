#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>

#include "SwerveDrive.h"
#include "Constants.h"

class PathManager {

    std::vector<frc::Trajectory> trajectories;

    public:
    PathManager();

    // handle holonomic drive controller to follow a passed in trajectory

};