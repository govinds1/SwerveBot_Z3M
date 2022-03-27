#pragma once

#include <unordered_map>
#include <string>
#include <utility>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "Constants.h"

class PathManager {

    std::unordered_map<std::string, frc::Trajectory> trajectories;

    public:
    PathManager();
    std::string ConcatPoseNames(std::string pose1, std::string pose2);
    void AddTrajectory(std::string name, frc::Trajectory trajectory);
    void AddTrajectory(std::string startPoseName, std::string endPoseName, frc::Trajectory trajectory);
    void GenTrajectory(const POSES::field_pose& start, const POSES::field_pose& end, const std::vector<POSES::field_pose>& waypoints = {}, units::feet_per_second_t max_vel = SPEEDS::MAX_CHASSIS_SPEED, units::feet_per_second_squared_t max_accel = SPEEDS::MAX_CHASSIS_ACCEL);


    frc2::PIDController xController{PID_VALUES::CHASSIS_X.P, PID_VALUES::CHASSIS_X.I, PID_VALUES::CHASSIS_X.D}; // adds control for forward-backward error
    frc2::PIDController yController{PID_VALUES::CHASSIS_Y.P, PID_VALUES::CHASSIS_Y.I, PID_VALUES::CHASSIS_Y.D}; // adds control for left-right error
    frc::ProfiledPIDController<units::radian> thetaController{ // adds control for azimuth error
        PID_VALUES::CHASSIS_ROT.P, PID_VALUES::CHASSIS_ROT.I, PID_VALUES::CHASSIS_ROT.D,
        frc::TrapezoidProfile<units::radian>::Constraints{SPEEDS::MAX_CHASSIS_TURN_SPEED, SPEEDS::MAX_CHASSIS_TURN_ACCEL}
    };
    frc::HolonomicDriveController swerveDriveController{xController, yController, thetaController};

    frc::ChassisSpeeds CalculateSpeeds(units::time::second_t currentTime, std::string trajectoryName, frc::Pose2d currentPose, frc::Rotation2d currentHeading);
    frc::ChassisSpeeds CalculateSpeeds(units::time::second_t currentTime, std::string startPoseName, std::string endPoseName, frc::Pose2d currentPose, frc::Rotation2d currentHeading);

    double TrajectoryStatus(units::time::second_t currentTime, std::string trajectoryName);

    bool AtTarget();
    void SetEnabled(bool enabled);
    

};