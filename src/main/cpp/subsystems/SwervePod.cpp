#include "subsystems/SwervePod.h"
#include "math.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>

SwervePod::SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor, int encoderChannel) : m_topMotor(topMotor), 
                                                                                  m_bottomMotor(bottomMotor), 
                                                                                  m_desiredYawDegrees(0),
                                                                                  m_mode(Pod_Off)
{
    m_podEncoder = new frc::DutyCycleEncoder(encoderChannel);
    m_podEncoder->SetDutyCycleRange(1, 1024);
}

void SwervePod::Periodic() {
    // Put code here to be run every loop
    m_currentTopMotorSpeed = m_topMotor->GetEncoder().GetVelocity();
    m_currentBottomMotorSpeed = m_bottomMotor->GetEncoder().GetVelocity();
    m_currentPosition = m_podEncoder->GetAbsolutePosition();
}

void SwervePod::SimulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

}

void SwervePod::Initialize() {
    m_podEncoder->Reset();
    m_initialized = true;
}

// TODO: Move to a different location
// Convert from absolute encoder position to degrees
double ToDegree(double pos) {

    double deg_value = pos/1024.0*360.0;

    return deg_value;
}

// Convert degrees to radians
double ToRadian(double x) {
    return x * (3.14159/180.0);
}

void SwervePod::Drive(frc::SwerveModuleState state) {
    //TODO: math to figure out what to set motor speeds to
    //TODO: gear ratio stuff
    double _topMotorSpeed = 0;
    double _bottomMotorSpeed = 0;
    double lesserAngleMargin;
    double greaterAngleMargin = 10;

    double current_angle = ToDegree(m_currentPosition);

    auto optimizedDesiredState = frc::SwerveModuleState::Optimize(state, units::radian_t(ToRadian(current_angle)));

    double delta = std::fabs(current_angle-(double)optimizedDesiredState.angle.Degrees());

    if (delta < lesserAngleMargin) {
        // basically correct, move forward
        m_topMotor->Set(optimizedDesiredState.speed.value());
        m_bottomMotor->Set(-optimizedDesiredState.speed.value());
    } else if (delta < greaterAngleMargin) {
        // close but not all the way, move and rotate

    } else {
        // angle isn't close, rotate only
        m_topMotor->Set(1);
        m_bottomMotor->Set(1);
    }

    switch(m_mode)
    {
        case Pod_Off:
        default:
          break;
        case Pod_Move:
          break;
        case Pod_Rotate:
          break;
        case Pod_RotateAndMove:
          break;
    }



    m_topMotor->Set(_topMotorSpeed);
    m_bottomMotor->Set(_bottomMotorSpeed);
}