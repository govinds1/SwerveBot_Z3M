#include "SwerveDrive.h"

SwerveDrive::SwerveDrive(std::shared_ptr<PathManager> pathManager) {

    m_pathManager = pathManager;
    fieldRelative = false;
    ResetSpeeds();
}

void SwerveDrive::Init(bool autonomous, frc::Pose2d initialPose) {
    // Use hand set Pose from selected auton to Set odometry here
    // If run in teleop init, or there is no selected auton, Pose should just be origin
    // do not reset pose in both teleop and auton, unless you know the exact poses
    if (autonomous) {
        SetPose(initialPose);
    }

    fieldRelative = false;
    ResetSpeeds();
    m_leftFront->Init();
    m_leftRear->Init();
    m_rightFront->Init();
    m_rightRear->Init();
}

void SwerveDrive::Periodic() {
    m_leftFront->Periodic();
    m_leftRear->Periodic();
    m_rightFront->Periodic();
    m_rightRear->Periodic();

    m_odometry.Update(
        GetAngle(), 
        m_leftFront->GetTrueState(),
        m_leftRear->GetTrueState(),
        m_rightFront->GetTrueState(),
        m_rightRear->GetTrueState()
    );
}

// Takes direct input from controller axes, handle conversions to real units and proper robot coordinates in this function
// Robot coords: 
//      forward = +x
//      right = -y
//      turn = -omega (CCW (turning left) is positive)
void SwerveDrive::Drive(double forward, double right, double turn) {
    SetDesiredSpeeds(
        forward * SPEEDS::MAX_FORWARD_SPEED,
        -right * SPEEDS::MAX_STRAFE_SPEED,
        -turn * SPEEDS::MAX_TURN_SPEED
    );
}

void SwerveDrive::Drive(double forward, double right, double turn, bool driveFieldRelative) {
    bool fieldRelSaved = fieldRelative;
    SetFieldRelative(driveFieldRelative);
    Drive(forward, right, turn);
    SetFieldRelative(fieldRelSaved);
}

double SwerveDrive::FollowTrajectory(units::time::second_t currentTime, std::string trajectoryName) {
    SetDesiredSpeeds(m_pathManager->CalculateSpeeds(currentTime, trajectoryName, m_odometry.GetPose(), GetAngle()));
    return m_pathManager->TrajectoryStatus(currentTime, trajectoryName);
}

double SwerveDrive::FollowTrajectory(units::time::second_t currentTime, std::string startPoseName, std::string endPoseName) {
    return FollowTrajectory(currentTime, m_pathManager->ConcatPoseNames(startPoseName, endPoseName));
}

void SwerveDrive::ResetSpeeds() {
    SetDesiredSpeeds(0, 0, 0);
}

// Zeroing options:
//  Manually -> Turn all wheels to face forward and then calibrate (store the absolute encoder position or set encoder position to 0)
//  Temporary Limit Switch -> Have a pin/limit switch so the wheel can spin until it hits it and stops (facing forward), then calibrate 
//  https://www.chiefdelphi.com/t/swerve-zeroing/181799

void SwerveDrive::CalibrateWheelsManually() {
    // Should only be called when all wheels are facing front (at the "zero" position)

    m_leftFront->CalibrateAngle();
    m_leftRear->CalibrateAngle();
    m_rightFront->CalibrateAngle();
    m_rightRear->CalibrateAngle();


    ENCODER_ZEROS::LEFT_FRONT = m_leftFront->GetRawAngle();
    ENCODER_ZEROS::LEFT_REAR = m_leftRear->GetRawAngle();
    ENCODER_ZEROS::RIGHT_FRONT = m_rightFront->GetRawAngle();
    ENCODER_ZEROS::RIGHT_REAR = m_rightRear->GetRawAngle();
    // ENCODER_ZEROS::SET_ZEROS(
    //     m_leftFront->GetRawAngle(), 
    //     m_leftRear->GetRawAngle(),
    //     m_rightFront->GetRawAngle(), 
    //     m_rightRear->GetRawAngle()
    // );
}


void SwerveDrive::SetDesiredSpeeds(double vx, double vy, double omega) {
    SetDesiredSpeeds(
        units::velocity::feet_per_second_t(vx),
        units::velocity::feet_per_second_t(vy),
        units::angular_velocity::radians_per_second_t(omega)
    );
}

void SwerveDrive::SetDesiredSpeeds(units::velocity::feet_per_second_t vx, units::velocity::feet_per_second_t vy, units::angular_velocity::radians_per_second_t omega) {
    if (fieldRelative) {
        SetDesiredSpeeds(frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, GetAngle()));
    } else {
        auto newSpeeds = frc::ChassisSpeeds();
        newSpeeds.vx = vx;
        newSpeeds.vy = vy;
        newSpeeds.omega = omega;
        SetDesiredSpeeds(newSpeeds);
    }
}

void SwerveDrive::SetDesiredSpeeds(frc::ChassisSpeeds newSpeeds) {
    m_desiredSpeeds = newSpeeds;
    SetWheelStates();
}

void SwerveDrive::SetTrueSpeeds() {
    m_trueSpeeds = m_kinematics.ToChassisSpeeds(
        m_leftFront->GetTrueState(),
        m_leftRear->GetTrueState(),
        m_rightFront->GetTrueState(),
        m_rightRear->GetTrueState()
    );
}

void SwerveDrive::SetWheelStates() {
    auto stateArray = m_kinematics.ToSwerveModuleStates(m_desiredSpeeds);
    m_kinematics.DesaturateWheelSpeeds(&stateArray, SPEEDS::MAX_FORWARD_SPEED + SPEEDS::MAX_STRAFE_SPEED);
    auto [lf, lr, rf, rr] = stateArray;
    m_leftFront->SetDesiredState(lf);
    m_leftRear->SetDesiredState(lr);
    m_rightFront->SetDesiredState(rf);
    m_rightRear->SetDesiredState(rr);
}

void SwerveDrive::SetFieldRelative(bool fieldRel) {
    fieldRelative = fieldRel;
}

void SwerveDrive::SetPose(frc::Pose2d newPose) {
    m_odometry.ResetPosition(newPose, GetAngle());
}

void SwerveDrive::SetPose(double x, double y) {
    SetPose(units::meter_t(x), units::meter_t(y));
}

void SwerveDrive::SetPose(units::meter_t x, units::meter_t y) {
    frc::Pose2d newPose = frc::Pose2d(x, y, GetAngle());
    SetPose(newPose);
}

void SwerveDrive::ResetGyro() {
    // Reset Gyro to a heading of 0 and SetPose so the new angle is updated in odometry
    m_gyro.Reset();
    SetPose(GetPose());
}

frc::Rotation2d SwerveDrive::GetAngle() {
    // Return Gyro reading
    // negate if necessary, turning left (CCW) should be positive
    return frc::Rotation2d(units::angle::degree_t(m_gyro.GetAngle()));
}

frc::Pose2d SwerveDrive::GetPose() {
    return m_odometry.GetPose();
}

frc::ChassisSpeeds SwerveDrive::GetDesiredSpeeds() {
    return m_desiredSpeeds;
}
frc::ChassisSpeeds SwerveDrive::GetTrueSpeeds() {
    return m_trueSpeeds;
}