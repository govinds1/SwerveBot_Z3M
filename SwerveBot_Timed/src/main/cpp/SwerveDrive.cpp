#include "SwerveDrive.h"

SwerveDrive::SwerveDrive() {
    fieldRelative = false;
    ResetSpeeds();
}

void SwerveDrive::Init() {
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
}

// Takes direct input from controller axes, handle conversions to real units and proper robot coordinates in this function
// Robot coords: 
//      forward = +x
//      right = -y
//      turn = +omega (CCW is positive)
void SwerveDrive::Drive(double forward, double right, double turn) {
    SetSpeeds(
        forward * SPEEDS::MAX_FORWARD_SPEED,
        -right * SPEEDS::MAX_STRAFE_SPEED,
        units::angular_velocity::radians_per_second_t(turn * SPEEDS::MAX_TURN_SPEED)
    );
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
    if (!fieldRelative) {
        m_desiredSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, GetAngle());
    } else {
        m_desiredSpeeds.vx = vx;
        m_desiredSpeeds.vy = vy;
        m_desiredSpeeds.omega = omega;
    }
}

void SwerveDrive::SetWheelStates() {
    auto [lf, lr, rf, rr] = m_kinematics.ToSwerveModuleStates(m_desiredSpeeds);
    m_leftFront.SetState(lf);
    m_leftRear.SetState(lr);
    m_rightFront.SetState(rf);
    m_rightRear.SetState(rr);
}

void SwerveDrive::SetFieldRelative(bool fieldRel) {
    fieldRelative = fieldRel;
}

frc::Rotation2d SwerveDrive::GetAngle() {
    // Return Gyro reading
    // return frc::Rotation2d(gyro.Get());

    return frc::Rotation2d(units::angle::radian_t(0));

}