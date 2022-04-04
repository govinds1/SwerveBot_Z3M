#include "Turret.h"


Turret::Turret() {
    SetTurretPID(PID_VALUES::TURRET.P, PID_VALUES::TURRET.I, PID_VALUES::TURRET.D, PID_VALUES::TURRET.F);

}

void Turret::Init() {

}

void Turret::Periodic() {
    
}

void Turret::SetTurretPID(double p, double i, double d, double f) {
    m_turret->Config_kP(0, p);
    m_turret->Config_kI(0, i);
    m_turret->Config_kD(0, d);
    m_turret->Config_kF(0, f);
}