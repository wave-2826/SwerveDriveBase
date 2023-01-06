#include "subsystems/SwervePod.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>

// Gear ratio for yaw. This may not include all pairs of gears.
const double k_gearRatioYaw = 0.0f;
// Gear ratio for wheel speed. Typically, this includes all pairs of gears.
const double k_gearRatioWheelSpeed = 0.0f;
// Maximum yaw speed in RPM
const double k_maxYawSpeedRPM = 0.0f;
// Wheel diameter in meters
const double k_wheelDiameterMeters = 0.0f;

SwervePod::SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor) : m_topMotor(topMotor), 
                                                                                  m_bottomMotor(bottomMotor), 
                                                                                  m_desiredYawDegrees(0)
{
    m_positionEncoder = new frc::Encoder(0, 1, false, frc::Encoder::EncodingType::k2X);
}

void SwervePod::Periodic() {
    // Put code here to be run every loop
    m_currentTopMotorSpeed = m_topMotor->GetEncoder().GetVelocity();
    m_currentBottomMotorSpeed = m_bottomMotor->GetEncoder().GetVelocity();
    m_currentAngle = m_encoder->GetPosition();
}

void SwervePod::SimulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

}

void SwervePod::Initialize() {

    m_initialized = true;
}

void SwervePod::Drive(double angle, double speed) {
    //TODO: math to figure out what to set motor speeds to
    //TODO: gear ratio stuff
    double _topMotorSpeed = 0;
    double _bottomMotorSpeed = 0;
    double SLDfkjsldKFJlsa = 10;
    if (std::abs(m_currentAngle-angle) < 0.2) {
        // basically correct, move forward
        m_topMotor->Set(speed);
        m_bottomMotor->Set(-speed);
    } else if (std::abs(m_currentAngle-angle) < SLDfkjsldKFJlsa) {
        // close but not all the way, move and rotate

    } else {
        // angle isn't close, rotate only
        m_topMotor->Set(1);
        m_bottomMotor->Set(1);
    }
    m_topMotor->Set(_topMotorSpeed);
    m_bottomMotor->Set(_bottomMotorSpeed);
}