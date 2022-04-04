#include "Shooter.h"


Shooter::Shooter() {
    SetShooterPID(PID_VALUES::SHOOTER.P, PID_VALUES::SHOOTER.I, PID_VALUES::SHOOTER.D, PID_VALUES::SHOOTER.F);

}

void Shooter::Init() {

}

void Shooter::Periodic() {

}

void Shooter::SetShooterPID(double p, double i, double d, double f) {
    m_shooter->Config_kP(0, p);
    m_shooter->Config_kI(0, i);
    m_shooter->Config_kD(0, d);
    m_shooter->Config_kF(0, f);
}