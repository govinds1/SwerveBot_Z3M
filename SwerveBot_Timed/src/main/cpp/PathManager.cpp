#include "PathManager.h"

PathManager::PathManager() {
    // Add Trajectories here
    AddTrajectory("Left Start to Ball 1", GenTrajectory(POSES::AUTON_LEFT_START, POSES::BALL_1));
    AddTrajectory("Ball 1 to Ball 2", GenTrajectory(POSES::BALL_1, POSES::BALL_2));
    AddTrajectory("Left Start to Ball 2", trajectories["Left Start to Ball 1"] + trajectories["Ball 1 to Ball 2"]);
}

void PathManager::AddTrajectory(std::string name, frc::Trajectory trajectory) {
    trajectories.insert(std::pair<std::string, frc::Trajectory>(name, trajectory));
}

frc::Trajectory PathManager::GenTrajectory(const POSES::field_pose &start, const POSES::field_pose &end, const std::vector<POSES::field_pose> &waypoints, units::feet_per_second_t max_vel, units::feet_per_second_squared_t max_accel) {
    std::vector<frc::Translation2d> wpts;
    for (auto wpt : waypoints) {
        wpts.push_back(wpt.toTranslation());
    }
    auto config = frc::TrajectoryConfig(max_vel, max_accel);
    // change config variables (startVel, endVel, reversed, constraints) if necessary
    return frc::TrajectoryGenerator::GenerateTrajectory(start.toPose(), wpts, end.toPose(), config);
}

frc::ChassisSpeeds PathManager::CalculateSpeeds(units::time::second_t currentTime, std::string trajectoryName, frc::Pose2d currentPose, frc::Rotation2d currentHeading) {
    auto desiredState = trajectories[trajectoryName].Sample(currentTime);
    return swerveDriveController.Calculate(currentPose, desiredState, currentHeading);
}