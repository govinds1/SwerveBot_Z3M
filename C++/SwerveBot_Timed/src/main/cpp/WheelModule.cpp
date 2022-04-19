#include "WheelModule.h"

WheelModule::WheelModule(std::shared_ptr<rev::CANSparkMax> driveMotor, std::shared_ptr<rev::CANSparkMax> angleMotor, 
            std::shared_ptr<rev::SparkMaxRelativeEncoder> driveEncoder, std::shared_ptr<rev::SparkMaxRelativeEncoder> angleEncoder,
            std::shared_ptr<rev::SparkMaxPIDController> drivePIDController, std::shared_ptr<rev::SparkMaxPIDController> anglePIDController,
            int angleEncZero) {
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

    m_angleEncZero = angleEncZero;

    ResetState();
}

void WheelModule::Init() {
    ResetState();
}

void WheelModule::Periodic() {
    // When using quadrature encoder (instead of NEO sensor) for angle, divide the state angle value by the conversion factor since SetReference will need the ref in ticks
    m_anglePIDController->SetReference((m_desiredState.angle.Radians().value() / m_angleEncoder->GetPositionConversionFactor()) + m_angleEncZero, rev::CANSparkMax::ControlType::kPosition);
    m_drivePIDController->SetReference(m_desiredState.speed.value(), rev::CANSparkMax::ControlType::kVelocity);
    SetTrueState();
}

void WheelModule::ResetState() {
    SetDesiredState(0.0, 0.0);
    m_desiredState.speed = 0_fps;
    ResetAngle();
}

void WheelModule::ResetAngle() {
    m_angleEncoder->SetPosition(0.0); // we can do this with the NEO relative encoder, calling this on robotInit assumes the wheels start facing forward (not good)
    m_desiredState.angle = frc::Rotation2d(0_rad);
    // reset gyro to 0
}

void WheelModule::CalibrateAngle() {
    // Call this function when the wheel faces forward, will set current encoder ticks as the zero variable
    m_angleEncZero = m_angleEncoder->GetPosition(); // should be in raw ticks when using quadrature encoder

    // for now, with NEO sensor, just set the current position to be 0
    m_angleEncoder->SetPosition(0);
    m_angleEncZero = 0;
}

void WheelModule::SetDesiredState(frc::SwerveModuleState newState) {
    m_desiredState = frc::SwerveModuleState::Optimize(newState, frc::Rotation2d(GetAngle()));
}

void WheelModule::SetDesiredState(double feetPerSec, double radians) {
    SetDesiredState(feetPerSec, frc::Rotation2d(units::angle::radian_t(radians)));
}

void WheelModule::SetDesiredState(double feetPerSec, frc::Rotation2d radians) {
    frc::SwerveModuleState newState;
    newState.speed = units::velocity::feet_per_second_t(feetPerSec);
    newState.angle = radians;
    SetDesiredState(newState);
}

void WheelModule::SetTrueState() {
    m_trueState.speed = GetSpeed();
    m_trueState.angle = GetAngle();
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

int WheelModule::GetRawAngle() {
    return (GetAngle() / m_angleEncoder->GetPositionConversionFactor()).value();
}

units::angle::radian_t WheelModule::GetAngle() {
    return units::angle::radian_t(m_angleEncoder->GetPosition() - (m_angleEncZero * m_angleEncoder->GetPositionConversionFactor()));
}

units::velocity::feet_per_second_t WheelModule::GetSpeed() {
    return units::velocity::feet_per_second_t(m_driveEncoder->GetVelocity());
}

units::length::foot_t WheelModule::GetDistance() {
    return units::length::foot_t(m_driveEncoder->GetPosition());
}

const frc::SwerveModuleState& WheelModule::GetDesiredState() {
    return m_desiredState;
}

const frc::SwerveModuleState& WheelModule::GetTrueState() {
    return m_trueState;
}