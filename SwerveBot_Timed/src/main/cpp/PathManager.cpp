#include "PathManager.h"

PathManager::PathManager() {
    // Add Trajectories here
    GenTrajectory(POSES::AUTON_LEFT_START, POSES::BALL_1);
    GenTrajectory(POSES::BALL_1, POSES::BALL_2);
    GenTrajectory(POSES::BALL_2, POSES::BALL_3);
    GenTrajectory(POSES::BALL_3, POSES::BALL_4);
    GenTrajectory(POSES::BALL_4, POSES::BALL_5);
    GenTrajectory(POSES::BALL_5, POSES::SHOOTING_SPOT_LEFT);
    GenTrajectory(POSES::BALL_5, POSES::SHOOTING_SPOT_MIDDLE);
    GenTrajectory(POSES::BALL_5, POSES::SHOOTING_SPOT_RIGHT);
    AddTrajectory(TRAJECTORIES::LEFT_START_TO_BALL_2, trajectories[TRAJECTORIES::LEFT_START_TO_BALL_1] + trajectories[TRAJECTORIES::BALL_1_TO_BALL_2]);

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

bool PathManager::IsTrajectoryFinished(units::time::second_t currentTime, std::string trajectoryName) {
    return trajectories[trajectoryName].TotalTime() > currentTime;
}