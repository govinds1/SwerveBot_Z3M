#include "PathManager.h"

PathManager::PathManager() {
    // Add Trajectories here
    std::vector<POSES::field_pose> center_waypoint = {POSES::CENTRAL_WAYPOINT};
    GenTrajectory(POSES::AUTON_LEFT_START, POSES::BALL_RIGHT, center_waypoint);
    GenTrajectory(POSES::AUTON_MIDDLE_START, POSES::BALL_RIGHT);
    GenTrajectory(POSES::AUTON_RIGHT_START, POSES::BALL_RIGHT);

    GenTrajectory(POSES::AUTON_LEFT_START, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::AUTON_MIDDLE_START, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::AUTON_RIGHT_START, POSES::BALL_MIDDLE);

    GenTrajectory(POSES::AUTON_LEFT_START, POSES::BALL_LEFT);
    GenTrajectory(POSES::AUTON_MIDDLE_START, POSES::BALL_LEFT);
    GenTrajectory(POSES::AUTON_RIGHT_START, POSES::BALL_LEFT, center_waypoint);

    GenTrajectory(POSES::AUTON_LEFT_START, POSES::BALL_HUMAN_PLAYER, center_waypoint);
    GenTrajectory(POSES::AUTON_MIDDLE_START, POSES::BALL_HUMAN_PLAYER, center_waypoint);
    GenTrajectory(POSES::AUTON_RIGHT_START, POSES::BALL_HUMAN_PLAYER, center_waypoint);

    GenTrajectory(POSES::BALL_RIGHT, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::BALL_RIGHT, POSES::BALL_LEFT, center_waypoint);
    GenTrajectory(POSES::BALL_RIGHT, POSES::BALL_HUMAN_PLAYER, center_waypoint);
    GenTrajectory(POSES::BALL_RIGHT, POSES::SHOOTING_SPOT_LEFT, center_waypoint);
    GenTrajectory(POSES::BALL_RIGHT, POSES::SHOOTING_SPOT_MIDDLE, center_waypoint);
    GenTrajectory(POSES::BALL_RIGHT, POSES::SHOOTING_SPOT_RIGHT);

    GenTrajectory(POSES::BALL_MIDDLE, POSES::BALL_RIGHT);
    GenTrajectory(POSES::BALL_MIDDLE, POSES::BALL_LEFT);
    GenTrajectory(POSES::BALL_MIDDLE, POSES::BALL_HUMAN_PLAYER);
    GenTrajectory(POSES::BALL_MIDDLE, POSES::SHOOTING_SPOT_LEFT);
    GenTrajectory(POSES::BALL_MIDDLE, POSES::SHOOTING_SPOT_MIDDLE);
    GenTrajectory(POSES::BALL_MIDDLE, POSES::SHOOTING_SPOT_RIGHT);

    GenTrajectory(POSES::BALL_LEFT, POSES::BALL_RIGHT);
    GenTrajectory(POSES::BALL_LEFT, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::BALL_LEFT, POSES::BALL_HUMAN_PLAYER);
    GenTrajectory(POSES::BALL_LEFT, POSES::SHOOTING_SPOT_LEFT);
    GenTrajectory(POSES::BALL_LEFT, POSES::SHOOTING_SPOT_MIDDLE);
    GenTrajectory(POSES::BALL_LEFT, POSES::SHOOTING_SPOT_RIGHT);

    GenTrajectory(POSES::BALL_HUMAN_PLAYER, POSES::BALL_RIGHT, center_waypoint);
    GenTrajectory(POSES::BALL_HUMAN_PLAYER, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::BALL_HUMAN_PLAYER, POSES::BALL_LEFT, center_waypoint);
    GenTrajectory(POSES::BALL_HUMAN_PLAYER, POSES::SHOOTING_SPOT_LEFT, center_waypoint);
    GenTrajectory(POSES::BALL_HUMAN_PLAYER, POSES::SHOOTING_SPOT_MIDDLE, center_waypoint);
    GenTrajectory(POSES::BALL_HUMAN_PLAYER, POSES::SHOOTING_SPOT_RIGHT, center_waypoint);

    GenTrajectory(POSES::SHOOTING_SPOT_LEFT, POSES::BALL_RIGHT, center_waypoint);
    GenTrajectory(POSES::SHOOTING_SPOT_LEFT, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::SHOOTING_SPOT_LEFT, POSES::BALL_LEFT);
    GenTrajectory(POSES::SHOOTING_SPOT_LEFT, POSES::BALL_HUMAN_PLAYER, center_waypoint);

    GenTrajectory(POSES::SHOOTING_SPOT_MIDDLE, POSES::BALL_RIGHT, center_waypoint);
    GenTrajectory(POSES::SHOOTING_SPOT_MIDDLE, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::SHOOTING_SPOT_MIDDLE, POSES::BALL_LEFT);
    GenTrajectory(POSES::SHOOTING_SPOT_MIDDLE, POSES::BALL_HUMAN_PLAYER, center_waypoint);

    GenTrajectory(POSES::SHOOTING_SPOT_RIGHT, POSES::BALL_RIGHT);
    GenTrajectory(POSES::SHOOTING_SPOT_RIGHT, POSES::BALL_MIDDLE);
    GenTrajectory(POSES::SHOOTING_SPOT_RIGHT, POSES::BALL_LEFT, center_waypoint);
    GenTrajectory(POSES::SHOOTING_SPOT_RIGHT, POSES::BALL_HUMAN_PLAYER, center_waypoint);

    //AddTrajectory(TRAJECTORIES::LEFT_START_TO_BALL_2, trajectories[TRAJECTORIES::LEFT_START_TO_BALL_1] + trajectories[TRAJECTORIES::BALL_1_TO_BALL_2]);

    swerveDriveController.SetTolerance(TRAJECTORIES::trajectoryTolerance);
}

std::string PathManager::ConcatPoseNames(std::string pose1, std::string pose2) {
    return pose1 + " to " + pose2;
}

void PathManager::AddTrajectory(std::string name, frc::Trajectory trajectory) {
    trajectories.insert(std::pair<std::string, frc::Trajectory>(name, trajectory));
}

void PathManager::AddTrajectory(std::string startPoseName, std::string endPoseName, frc::Trajectory trajectory) {
    trajectories.insert(std::pair<std::string, frc::Trajectory>(ConcatPoseNames(startPoseName, endPoseName), trajectory));
}

void PathManager::GenTrajectory(const POSES::field_pose &start, const POSES::field_pose &end, const std::vector<POSES::field_pose> &waypoints, units::feet_per_second_t max_vel, units::feet_per_second_squared_t max_accel) {
    std::vector<frc::Translation2d> wpts;
    for (auto wpt : waypoints) {
        wpts.push_back(wpt.toTranslation());
    }
    auto config = frc::TrajectoryConfig(max_vel, max_accel);
    // change config variables (startVel, endVel, reversed, constraints) if necessary
    AddTrajectory(start.NAME, end.NAME, frc::TrajectoryGenerator::GenerateTrajectory(start.toPose(), wpts, end.toPose(), config));
}

frc::ChassisSpeeds PathManager::CalculateSpeeds(units::time::second_t currentTime, std::string trajectoryName, frc::Pose2d currentPose, frc::Rotation2d currentHeading) {
    auto desiredState = trajectories[trajectoryName].Sample(currentTime);
    return swerveDriveController.Calculate(currentPose, desiredState, currentHeading);
}

frc::ChassisSpeeds PathManager::CalculateSpeeds(units::time::second_t currentTime, std::string startPoseName, std::string endPoseName, frc::Pose2d currentPose, frc::Rotation2d currentHeading) {
    auto desiredState = trajectories[ConcatPoseNames(startPoseName, endPoseName)].Sample(currentTime);
    return swerveDriveController.Calculate(currentPose, desiredState, currentHeading);
}

double PathManager::TrajectoryStatus(units::time::second_t currentTime, std::string trajectoryName) {
    return (currentTime / trajectories[trajectoryName].TotalTime());
}

bool PathManager::AtTarget() {
    return swerveDriveController.AtReference();
}

void PathManager::SetEnabled(bool enabled) {
    swerveDriveController.SetEnabled(enabled);
}