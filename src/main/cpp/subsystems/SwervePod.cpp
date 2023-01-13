#include "subsystems/SwervePod.h"
#include "math.h"
#include <iostream>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Rotation2d.h>

SwervePod::SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor, int encoderChannel) : m_topMotor(topMotor), 
                                                                                  m_bottomMotor(bottomMotor), 
                                                                                  m_desiredYawDegrees(0),
                                                                                  m_podOperationMode(Pod_Off)
{
    m_topEncoder = new rev::SparkMaxRelativeEncoder(m_topMotor->GetEncoder());
    m_bottomEncoder = new rev::SparkMaxRelativeEncoder(m_bottomMotor->GetEncoder());

    m_podEncoder = new frc::DutyCycleEncoder(encoderChannel);
    m_podEncoder->SetDutyCycleRange(1, 1024);
}

void SwervePod::Periodic() {
    // Put code here to be run every loop
    // std::cout << "Top:    " << m_currentTopMotorSpeed << std::endl;
    // std::cout << "Bottom: " << m_currentBottomMotorSpeed << std::endl;
}

void SwervePod::SimulationPeriodic() {
    // This method will be called once per scheduler run when in simulation

}

void SwervePod::Initialize() {
    m_podEncoder->Reset();
    m_initialized = true;
}

// TODO: Move to a conversion/util file
// Convert from absolute encoder position to degrees
double ToDegree(double pos) {

    double deg_value = pos/1024.0*360.0;

    return deg_value;
}

// TODO: Move to a conversion/util file
// Convert degrees to radians
double ToRadian(double degrees) {
    return degrees * (3.14159/180.0);
}

void SwervePod::Drive(frc::SwerveModuleState state, double xValue,double yValue) {
    //TODO: math to figure out what to set motor speeds to
    //TODO: gear ratio stuff

    //TODO: set max motor speeds
    double topMotorSpeed = 0;
    double bottomMotorSpeed = 0;

    double k_stopMargin = 0.01;
    double k_lesserAngleMargin = 0.2;
    double k_greaterAngleMargin = 0.5;
    double k_reductionFactor = 0.2;

    double current_angle = ToDegree(m_currentPosition);

    // optimize state (speed, angle) by minimizing change in heading and possibly reversing speed
    auto optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(ToRadian(current_angle)));
    double commanded_speed = (optimizedState.speed.value() / (5200.0 * k_gearRatioYaw));
    if(1 < commanded_speed)
    {
        commanded_speed = 1;
    }


    if((yValue > 0) || (yValue < 0)) {
        topMotorSpeed = (xValue - yValue);
        bottomMotorSpeed = xValue + yValue;
    } else {
        if((xValue > 0) || (xValue < 0)) {
            topMotorSpeed = xValue;
            bottomMotorSpeed = xValue;
        }
    }


    m_topMotor->Set(topMotorSpeed * k_reductionFactor);
    m_bottomMotor->Set(bottomMotorSpeed * k_reductionFactor);


    //std::cout << "Connected: " << m_podEncoder->IsConnected() << std::endl;
    //std::cout << "Channel: " << m_podEncoder->GetSourceChannel() << std::endl;
    //std::cout << "Position: " << m_podEncoder->GetAbsolutePosition() << std::endl;
 
    std::cout << "X: " << xValue << "Y: " << yValue << " Top: " << topMotorSpeed*k_reductionFactor << "  Bottom: " << bottomMotorSpeed*k_reductionFactor << std::endl << std::endl;    


}
