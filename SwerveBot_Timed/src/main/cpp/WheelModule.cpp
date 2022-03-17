#include "WheelModule.h"

WheelModule::WheelModule(std::shared_ptr<rev::CANSparkMax> driveMotor, std::shared_ptr<rev::CANSparkMax> angleMotor) {
    m_driveMotor = driveMotor;
    m_angleMotor = angleMotor;
}