#include "WheelModule.h"

WheelModule::WheelModule(std::shared_ptr<rev::CANSparkMax> driveMotor, std::shared_ptr<rev::CANSparkMax> angleMotor, 
            std::shared_ptr<rev::SparkMaxRelativeEncoder> driveEncoder, std::shared_ptr<rev::SparkMaxRelativeEncoder> angleEncoder,
            std::shared_ptr<rev::SparkMaxPIDController> drivePIDController, std::shared_ptr<rev::SparkMaxPIDController> anglePIDController) {
    m_driveMotor = driveMotor;
    m_angleMotor = angleMotor;
    m_driveEncoder = driveEncoder;
    m_angleEncoder = angleEncoder;
    m_drivePIDController = drivePIDController;
    m_anglePIDController = anglePIDController;

    m_driveEncoder->SetPositionConversionFactor(ENCODER_CONVERSIONS::DRIVE_FEET_PER_TICK);
    m_driveEncoder->SetVelocityConversionFactor(ENCODER_CONVERSIONS::DRIVE_FEET_PER_SECOND_PER_TICK);
    m_angleEncoder->SetPositionConversionFactor(ENCODER_CONVERSIONS::ANGLE_RAD_PER_TICK);

    SetDrivePID(PID_VALUES::DRIVE.P, PID_VALUES::DRIVE.I, PID_VALUES::DRIVE.D);
    SetAnglePID(PID_VALUES::ANGLE.P, PID_VALUES::ANGLE.I, PID_VALUES::ANGLE.D);

    ResetState();
}

void WheelModule::Init() {
    ResetState();
}

void WheelModule::Periodic() {
    m_drivePIDController->SetReference(m_state.speed.value(), rev::CANSparkMax::ControlType::kVelocity);
    m_anglePIDController->SetReference(m_state.angle.Radians().value(), rev::CANSparkMax::ControlType::kPosition);
}

void WheelModule::ResetState() {
    SetState(0.0, 0.0);
    m_state.speed = 0_fps;
    ResetAngle();
}

void WheelModule::ResetAngle() {
    m_angleEncoder->SetPosition(0.0);
    m_state.angle = frc::Rotation2d(0_rad);
}

void WheelModule::SetState(frc::SwerveModuleState newState) {
    m_state = frc::SwerveModuleState::Optimize(newState, frc::Rotation2d(GetAngle()));
}

void WheelModule::SetState(double feetPerSec, double radians) {
    SetState(feetPerSec, frc::Rotation2d(units::angle::radian_t(radians)));
}

void WheelModule::SetState(double feetPerSec, frc::Rotation2d radians) {
    frc::SwerveModuleState newState;
    newState.speed = units::velocity::feet_per_second_t(feetPerSec);
    newState.angle = radians;
    SetState(newState);
}

void WheelModule::SetDrivePID(double p, double i, double d) {
    m_drivePIDController->SetP(p);
    m_drivePIDController->SetI(i);
    m_drivePIDController->SetD(d);
}

void WheelModule::SetAnglePID(double p, double i, double d) {
    m_anglePIDController->SetP(p);
    m_anglePIDController->SetI(i);
    m_anglePIDController->SetD(d);
}

units::angle::radian_t WheelModule::GetAngle() {
    return units::angle::radian_t(m_angleEncoder->GetPosition());
}

units::velocity::feet_per_second_t WheelModule::GetSpeed() {
    return units::velocity::feet_per_second_t(m_driveEncoder->GetVelocity());
}

units::length::foot_t WheelModule::GetDistance() {
    return units::length::foot_t(m_driveEncoder->GetPosition());
}

const frc::SwerveModuleState& WheelModule::GetState() {
    return m_state;
}