#include "Turret.h"


Turret::Turret(std::shared_ptr<SwerveDrive> drive) {
    m_drive = drive;
    SetTurretPID(PID_VALUES::TURRET.P, PID_VALUES::TURRET.I, PID_VALUES::TURRET.D, PID_VALUES::TURRET.F);
    SetShooterPID(PID_VALUES::SHOOTER.P, PID_VALUES::SHOOTER.I, PID_VALUES::SHOOTER.D, PID_VALUES::SHOOTER.F);
}

void Turret::Init() {
    startAngle = m_drive->GetAngle(); // make sure this happens after setting start angle in drive init
    m_turret->SetSelectedSensorPosition(0);
}

void Turret::Periodic() {
    
}

// timeToShoot is the time until the ball will be shot
bool Turret::ShootAtHub() {
    frc::Pose2d currentPose = m_drive->GetPose();
    double x = currentPose.X().value(); // forward
    double y = currentPose.Y().value(); // left
    double theta = std::atan2(x, y);
    frc::Rotation2d setpoint = frc::Rotation2d(units::angle::radian_t(270 - theta)); // 270 - theta because CCW is positive
    frc::Rotation2d limelightOffset = frc::Rotation2d(units::angle::degree_t(limelight->GetNumber("tx", 0.0)));
    frc::Rotation2d limelightSetpoint = GetAngle() + limelightOffset;

    double distanceToGoal = std::sqrt(x * x + y * y);
    units::velocity::feet_per_second_t vball = SHOOTER::SHOOTER_BALL_LINEAR_LAUNCH_VELOCITY_AT_1_FOOT * distanceToGoal; // linear launch velocity of the ball without accounting for chassis speed

    // account for current speeds -> check notes for math
    auto currentSpeeds = m_drive->GetTrueSpeeds();
    units::velocity::feet_per_second_t vx = currentSpeeds.vx;
    units::velocity::feet_per_second_t vy = -currentSpeeds.vy; // negating for math reasons
    double v = std::sqrt(std::pow(vx.value(), 2) + std::pow(vy.value(), 2)); // total chassis speed
    double gammaAngle = (M_PI / 2.0) - std::atan2(vx.value(), vy.value()); // angle of chassis velocity off of 0
    vball = units::velocity::feet_per_second_t(std::sqrt(std::pow(vball.value(), 2) + std::pow(v, 2) - 2*(vball.value())*(v)*std::cos(gammaAngle))); // law of cosines
    frc::Rotation2d alpha = frc::Rotation2d(units::angle::radian_t(std::asin(std::sin(gammaAngle) * v) / vball.value())); // law of sines
    setpoint = setpoint + alpha;
    limelightSetpoint = limelightSetpoint + alpha;

    RunShooter(vball);

    if (!GoToAngle(setpoint)) {
        return false;
    } else if (!GoToAngle(limelightSetpoint)) {
        return false;
    }
    m_turret->Set(TalonFXControlMode::Velocity, 0.0);
    return (m_shooter->GetSelectedSensorVelocity() * SHOOTER::SHOOTER_RPM_CONVERSION * SHOOTER::SHOOTER_RPM_TO_BALL_LINEAR_LAUNCH_VELOCITY_CONVERSION) == vball.value();
    
}

bool Turret::GoToAngle(frc::Rotation2d angleToTurnTo) {
    // angleToTurn must be the setpoint angle with 0 facing opponent's alliance station
    // CCW is positive
    // subtract startAngle (or maybe current robot heading, if the shooter rotates with the chassis)
    angleToTurnTo = angleToTurnTo - startAngle;

    // optimize angle to closest, and take into account current encoder units (current num rotations could be way more than 1)
    angleToTurnTo = NormalizeAngle(angleToTurnTo); // normalize given angle
    if (AtAngle(angleToTurnTo)) return true;

    frc::Rotation2d currentAngle = GetAngle(); // get current angle, normalized
    frc::Rotation2d deltaAngle = angleToTurnTo - currentAngle; // get displacement from setpoint
    TurnAngle(deltaAngle);
    return false;
}

void Turret::TurnAngle(frc::Rotation2d deltaAngle) {
    deltaAngle = NormalizeAngle(deltaAngle); // make sure it's optimized
    double newEncoderSetpoint = m_turret->GetSelectedSensorPosition() + AngleToEncoder(deltaAngle);
    m_turret->Set(TalonFXControlMode::Position, newEncoderSetpoint);
}

bool Turret::AtAngle(frc::Rotation2d setpoint) {
    frc::Rotation2d error = GetAngle() - NormalizeAngle(setpoint);
    return (std::abs(error.Degrees().value()) < 0.5);
}


void Turret::SetTurretPID(double p, double i, double d, double f) {
    m_turret->Config_kP(0, p);
    m_turret->Config_kI(0, i);
    m_turret->Config_kD(0, d);
    m_turret->Config_kF(0, f);
}

void Turret::SetShooterPID(double p, double i, double d, double f) {
    m_shooter->Config_kP(0, p);
    m_shooter->Config_kI(0, i);
    m_shooter->Config_kD(0, d);
    m_shooter->Config_kF(0, f);
}

frc::Rotation2d Turret::GetAngle() {
    return NormalizeAngle(EncoderToAngle(m_turret->GetSelectedSensorPosition()));
}

frc::Rotation2d Turret::EncoderToAngle(double encoderUnits) {
    double numRotations = encoderUnits * SHOOTER::TURRET_TICKS_TO_ROTATION_CONVERSION;
    return frc::Rotation2d(numRotations * units::angle::degree_t(360.0));
}

double Turret::AngleToEncoder(frc::Rotation2d angle) {
    double numRotations = (angle.Degrees().value() / 360.0);
    numRotations = std::fmod(numRotations, 1.0);
    return numRotations / SHOOTER::TURRET_TICKS_TO_ROTATION_CONVERSION;
}

// Normalize given angle to [-180, 180]
frc::Rotation2d Turret::NormalizeAngle(frc::Rotation2d angle) {
    double newDegrees = angle.Degrees().value();
    newDegrees = std::fmod(newDegrees, 360.0);
    if (newDegrees > 180.0) {
        newDegrees -= 360.0;
    }
    return frc::Rotation2d(units::angle::degree_t(newDegrees));
}

void Turret::RunShooter(units::velocity::feet_per_second_t ballLinearLaunchVelocity) {
    RunShooter(ballLinearLaunchVelocity / SHOOTER::SHOOTER_RPM_TO_BALL_LINEAR_LAUNCH_VELOCITY_CONVERSION);
}

void Turret::RunShooter(units::angular_velocity::revolutions_per_minute_t rpm) {
    m_shooter->Set(TalonFXControlMode::Velocity, rpm.value() / SHOOTER::SHOOTER_RPM_CONVERSION);
}

units::angular_velocity::revolutions_per_minute_t Turret::GetRPM() {
    return units::angular_velocity::revolutions_per_minute_t(m_shooter->GetSelectedSensorVelocity() * SHOOTER::SHOOTER_RPM_CONVERSION);
}