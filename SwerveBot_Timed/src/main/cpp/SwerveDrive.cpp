#include "SwerveDrive.h"

SwerveDrive::SwerveDrive() {


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
    m_leftFront.Init();
    m_leftRear.Init();
    m_rightFront.Init();
    m_rightRear.Init();
}

void SwerveDrive::Periodic() {
    m_leftFront.Periodic();
    m_leftRear.Periodic();
    m_rightFront.Periodic();
    m_rightRear.Periodic();

    m_odometry.Update(
        GetAngle(), 
        m_leftFront.GetState(),
        m_leftRear.GetState(),
        m_rightFront.GetState(),
        m_rightRear.GetState()
    );
}

// Takes direct input from controller axes, handle conversions to real units and proper robot coordinates in this function
// Robot coords: 
//      forward = +x
//      right = -y
//      turn = -omega (CCW (turning left) is positive)
void SwerveDrive::Drive(double forward, double right, double turn) {
    SetSpeeds(
        forward * SPEEDS::MAX_FORWARD_SPEED,
        -right * SPEEDS::MAX_STRAFE_SPEED,
        units::angular_velocity::radians_per_second_t(-turn * SPEEDS::MAX_TURN_SPEED)
    );
}

void SwerveDrive::Drive(double forward, double right, double turn, bool driveFieldRelative) {
    bool fieldRelSaved = fieldRelative;
    SetFieldRelative(driveFieldRelative);
    Drive(forward, right, turn);
    SetFieldRelative(fieldRelSaved);
}

void SwerveDrive::ResetSpeeds() {
    SetSpeeds(0, 0, 0);
}

void SwerveDrive::SetSpeeds(double vx, double vy, double omega) {
    SetSpeeds(
        units::velocity::feet_per_second_t(vx),
        units::velocity::feet_per_second_t(vy),
        units::angular_velocity::radians_per_second_t(omega)
    );
}

void SwerveDrive::SetSpeeds(units::velocity::feet_per_second_t vx, units::velocity::feet_per_second_t vy, units::angular_velocity::radians_per_second_t omega) {
    if (fieldRelative) {
        m_desiredSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, GetAngle());
    } else {
        m_desiredSpeeds.vx = vx;
        m_desiredSpeeds.vy = vy;
        m_desiredSpeeds.omega = omega;
    }
    SetWheelStates();
}

void SwerveDrive::SetWheelStates() {
    auto stateArray = m_kinematics.ToSwerveModuleStates(m_desiredSpeeds);
    m_kinematics.DesaturateWheelSpeeds(&stateArray, SPEEDS::MAX_FORWARD_SPEED + SPEEDS::MAX_STRAFE_SPEED);
    auto [lf, lr, rf, rr] = stateArray;
    m_leftFront.SetState(lf);
    m_leftRear.SetState(lr);
    m_rightFront.SetState(rf);
    m_rightRear.SetState(rr);
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

    // gyro.Reset();
    SetPose(GetPose());
}

frc::Rotation2d SwerveDrive::GetAngle() {
    // Return Gyro reading
    // return frc::Rotation2d(gyro.Get());
    // negate if necessary, turning left (CCW) should be positive

    return -frc::Rotation2d(units::angle::radian_t(0));

}

frc::Pose2d SwerveDrive::GetPose() {
    return m_odometry.GetPose();
}