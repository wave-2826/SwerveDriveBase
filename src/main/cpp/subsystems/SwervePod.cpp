#include "subsystems/SwervePod.h"
#include "math.h"
#include <iostream>
#include <string>

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

    // DIO - returns val as a ratio of hi-time v low-time
    m_podEncoder = new frc::DutyCycleEncoder(encoderChannel);
    m_podEncoder->SetConnectedFrequencyThreshold(975.6);
    // range MUST be between 0 and 1 - setting results in NAN
    // m_podEncoder->SetDutyCycleRange(1/1024, 1023/1024);    
}

// TODO: Move to a conversion/util file
// Convert from absolute encoder position to degrees
double ToDegree(double pos) {

    // double deg_value = pos/1024.0*360.0;
    double deg_value = pos*360.0;

    return deg_value;
}

// TODO: Move to a conversion/util file
// Convert degrees to radians
double ToRadian(double degrees) {
    return degrees * (3.14159/180.0);
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
    // 0 is mechanical - offsets will have to be set in code - can't use Reset()
    // m_podEncoder->Reset();
    m_initialized = true;
    m_counter = 0;
    m_isReversed = false;
}

int SwervePod::GetCounter() {
    return m_counter;
}

void SwervePod::SetCounter(int count) {
    m_counter = count;
}

bool SwervePod::GetIsReversed() {
    return m_isReversed;
}

void SwervePod::FlipIsReversed(bool state) {
    m_counter = !state;
}

void SwervePod::Drive(frc::SwerveModuleState state, double xValue,double yValue) {
    //TODO: set max motor speeds
    double topMotorSpeed = 0;
    double bottomMotorSpeed = 0;

    double topMotorSpeedMove = 0;
    double bottomMotorSpeedMove = 0;
    double topMotorSpeedRotate = 0;
    double bottomMotorSpeedRotate = 0;

    double k_stopMargin = 0.01;
    double k_lesserAngleMargin = 0.2;
    double k_greaterAngleMargin = 0.5;
    double k_reductionFactor = 0.2;

    // tracking angles
    double current_angle = ToDegree(m_podEncoder->GetAbsolutePosition());
    double desired_angle = state.angle.Degrees().value() + 180;

    // optimize state (speed, angle) by minimizing change in heading and possibly reversing speed
    // state angle (desired) is -180 to 180
    // encoder angle (current) is 0 - 360
    auto optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(ToRadian(current_angle)));
    double commanded_speed = (state.speed.value() / 5200.0);

    if (1 < commanded_speed)
    {
        commanded_speed = 1;
    }

    // double angle_delta = fabs(current_angle - desired_angle);
    double angle_delta = desired_angle - current_angle;
    double angle_delta_optimized = 0.0;
    double tuning_value = 1 / 180.0;
    std::string swerveCase = "DID NOT ENTER";

    // TODO: fix the thing
    // If < 45 && not reversed
    // If > 45 && reversed
    if (fabs(angle_delta) <= 45.0 || fabs(angle_delta) >= 135.0) {
        // check if aligned - current angle within margin of error
        // TODO: Station keep
        swerveCase = "ALIGNED";
        // topMotorSpeed = commanded_speed * (1 + 0.05);
        // bottomMotorSpeed = commanded_speed * (1 - 0.05);
        double stationKeepTop = (1 - angle_delta * tuning_value);
        double stationKeepBottom = (1 + angle_delta * tuning_value);
        if(GetIsReversed()) {
            stationKeepTop = (1 + angle_delta * tuning_value);
            stationKeepBottom = (1 - angle_delta * tuning_value);
        }
        double divisor = 1;
        if (fabs(stationKeepTop) > fabs(stationKeepBottom)) {
            divisor = stationKeepTop / stationKeepBottom;
        } else {
            divisor = stationKeepBottom / stationKeepTop;
        }
        if (GetIsReversed()) {
            topMotorSpeed = commanded_speed * (stationKeepTop  / divisor);
            bottomMotorSpeed = commanded_speed * (stationKeepBottom / divisor);
        } else {
            topMotorSpeed = -commanded_speed * (stationKeepTop  / divisor);
            bottomMotorSpeed = -commanded_speed * (stationKeepBottom / divisor);
        }
        
    } else if (angle_delta < -90.0) {
        // check optimal path
        swerveCase = "CASE: OPTIMIZE < -90";
        angle_delta_optimized = -(angle_delta + 180.0);
        FlipIsReversed(m_isReversed);
        if (!GetIsReversed()) {
            topMotorSpeed = -commanded_speed * angle_delta_optimized * tuning_value;
            bottomMotorSpeed = commanded_speed  * angle_delta_optimized * tuning_value;  
        } else {
            topMotorSpeed = commanded_speed * angle_delta_optimized * tuning_value;
            bottomMotorSpeed = -commanded_speed  * angle_delta_optimized * tuning_value;
        }        
    } else if (angle_delta > 90.0) {
        // check optimal path
        swerveCase = "CASE: OPTIMIZE > 90";
        angle_delta_optimized = -(angle_delta - 180.0);
        FlipIsReversed(m_isReversed);
        if(!GetIsReversed()) {
            topMotorSpeed = -commanded_speed * angle_delta_optimized * tuning_value;
            bottomMotorSpeed = commanded_speed  * angle_delta_optimized * tuning_value;
        } else {
            topMotorSpeed = commanded_speed * angle_delta_optimized * tuning_value;
            bottomMotorSpeed = -commanded_speed  * angle_delta_optimized * tuning_value;
        }        
    } 
    else {
        swerveCase = "ROTATE";
        if(!GetIsReversed()) {
            topMotorSpeed = -commanded_speed * angle_delta * tuning_value;
            bottomMotorSpeed = commanded_speed * angle_delta * tuning_value;
        } else {
            // if (angle_delta < 0) {
            //     angle_delta_optimized = -(angle_delta - 180.0);
            // }
            // else {
            //     angle_delta_optimized = -(angle_delta + 180.0);
            // }
            
            topMotorSpeed = commanded_speed * angle_delta_optimized * tuning_value;
            bottomMotorSpeed = -commanded_speed * angle_delta_optimized * tuning_value;
        }        
    }

    m_topMotor->Set(topMotorSpeed);
    m_bottomMotor->Set(bottomMotorSpeed);

    ///////////////////////////////// TESTING PRINTOUTS ///////////////////////////////////////////////

    if (GetCounter() > 100) {

        std::cout << std::endl;
        std::cout << swerveCase << std::endl;
        std::cout << "top motor: " << topMotorSpeed << std::endl;
        std::cout << "bottom motor: " << bottomMotorSpeed << std::endl;

        std::cout << "commanded speed: " << commanded_speed << std::endl;
        std::cout << "tuning value: " << tuning_value << std::endl;
        // commanded_speed * angle_delta_optimized * tuning_value

        // std::cout << std::endl << "COMMANDED SPEED: " << commanded_speed << std::endl << std::endl;

        std::cout << std::endl;

        std::cout << "angle delta: " << angle_delta << std::endl;
        std::cout << "current angle: " << current_angle << std::endl;  
        std::cout << "desired angle: " << desired_angle << std::endl;   
        std::cout << "optimized angle delta: " << angle_delta_optimized << std::endl;    

        // std::cout << "angle delta: " << angle_delta << std::endl;
        // std::cout << "tuning value : " << tuning_value << std::endl;
        // std::cout << "Optimized desired Angle: " << optimizedState.angle.Degrees().value() << std::endl;

        // std::cout << "Degrees: " << ToDegree(m_podEncoder->GetAbsolutePosition()) << std::endl;
        // std::cout << "Radian: " << ToRadian(ToDegree(m_podEncoder->GetAbsolutePosition())) << std::endl;

        // std::cout << "State Speed: " << state.speed.value() << std::endl;
        // std::cout << "State Angle: " << state.angle.Degrees().value() << std::endl;

        // std::cout << "Optimized Speed: " << optimizedState.speed.value() << std::endl;
        // std::cout << "Optimized Angle: " << optimizedState.angle.Degrees().value() << std::endl;
    
        // std::cout << "X: " << xValue << "Y: " << yValue << " Top: " << topMotorSpeed*k_reductionFactor << "  Bottom: " << bottomMotorSpeed*k_reductionFactor << std::endl << std::endl;    
        SetCounter(0);
    } else {
      int current_count = GetCounter();
      SetCounter(current_count + 1);  
    }

}
