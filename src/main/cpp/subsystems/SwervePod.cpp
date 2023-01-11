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

void SwervePod::Drive(frc::SwerveModuleState state, uint8_t direction) {
    //TODO: math to figure out what to set motor speeds to
    //TODO: gear ratio stuff

    //TODO: set max motor speeds
    double topMotorSpeed = 0;
    double bottomMotorSpeed = 0;

    double k_stopMargin = 0.01;
    double k_lesserAngleMargin = 0.2;
    double k_greaterAngleMargin = 0.5;

    double current_angle = ToDegree(m_currentPosition);

    // optimize state (speed, angle) by minimizing change in heading and possibly reversing speed
    auto optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(ToRadian(current_angle)));
    double commanded_speed = (optimizedState.speed.value() / (5200.0 * k_gearRatioYaw));
    if(1 < commanded_speed)
    {
        commanded_speed = 1;
    }

    switch(direction)
    {
        case 0: // Off
        default:
            break;
        case 1: // Forward
            topMotorSpeed = -commanded_speed;
            bottomMotorSpeed = -commanded_speed;
            break;
        case 2: // Backward
            topMotorSpeed = commanded_speed;
            bottomMotorSpeed = commanded_speed;
            break;
    }

    m_topMotor->Set(topMotorSpeed);
    m_bottomMotor->Set(-bottomMotorSpeed);

/******

    // Check how far from the desired angle
    double delta = (current_angle - (double)optimizedState.angle.Degrees());
    double unsignedDelta = std::fabs(delta);

    // Debug info
    //std::printf("position: %f",m_currentPosition); 
    // std::printf("angles: %f, %f",current_angle, (double)optimizedState.angle.Degrees()); 
    //std::cout << std::endl;

    switch(m_podOperationMode)
    {
        case Pod_Off:
        default:
            if ((k_stopMargin <= unsignedDelta) && (unsignedDelta < k_lesserAngleMargin)){
                // close to desired angle, move forward
                m_podOperationMode = Pod_Move;
            } else if (unsignedDelta > k_greaterAngleMargin) {
                // Way from desired angle rotate then move
                m_podOperationMode = Pod_Rotate;
            }
            break;
        case Pod_Move:
            topMotorSpeed = optimizedState.speed.value();
            bottomMotorSpeed = -optimizedState.speed.value();

            if (k_greaterAngleMargin < unsignedDelta){
                // desired angle changed while moving, rotate then move
                m_podOperationMode = Pod_Rotate;
            }

            if((k_stopMargin >= unsignedDelta) || (0 == optimizedState.speed.value())){
                m_podOperationMode = Pod_Off;
            }
            break;
        case Pod_Rotate:
            if(0 < delta){
                // Turn CW
                topMotorSpeed = optimizedState.speed.value();
                bottomMotorSpeed = optimizedState.speed.value();
            } else{
                // Turn CCW
                topMotorSpeed = -optimizedState.speed.value();
                bottomMotorSpeed = -optimizedState.speed.value();
            }

            // close to desired angle, move forward
            if(k_lesserAngleMargin > unsignedDelta){
                m_podOperationMode = Pod_Move;
            }

            if((k_stopMargin >= unsignedDelta) || (0 == optimizedState.speed.value())){
                m_podOperationMode = Pod_Off;
            }
          break;
///        case Pod_RotateAndMove:
//            topMotorSpeed = optimizedState.speed.value();
//            bottomMotorSpeed = -optimizedState.speed.value();
//
//            if(k_stopMargin >= unsignedDelta)
//            {
//                m_podOperationMode = Pod_Off;
//            }
//          break;
    }

    // Debug info
    // std::printf("motor speed: %f, %f",topMotorSpeed, bottomMotorSpeed); 
    // std::cout << std::endl;

    //double cmdSpeed = optimizedState.speed.value() * 0.0001;
    //m_topMotor->Set(cmdSpeed);
    //m_bottomMotor->Set(-cmdSpeed);

    // m_topMotor->Set(topMotorSpeed);
    // m_bottomMotor->Set(bottomMotorSpeed);

******/
    //std::cout << "Connected: " << m_podEncoder->IsConnected() << std::endl;
    //std::cout << "Channel: " << m_podEncoder->GetSourceChannel() << std::endl;
    //std::cout << "Position: " << m_podEncoder->GetAbsolutePosition() << std::endl;
    std::cout << "Speed Cmd: " << commanded_speed << std::endl;

}

void SwervePod::Drive(frc::SwerveModuleState state, double xValue, double yValue){
    // m_currentPosition = m_podEncoder->GetAbsolutePosition();

    // std::cout << "Get:       " << m_podEncoder->Get() << std::endl;
    // std::cout << "GetAbsPos: " << m_podEncoder->GetAbsolutePosition() << std::endl;
    // std::cout << "GetDist:   " << m_podEncoder->GetDistance() << std::endl;
    // std::cout << "Dist/Rot:  " << m_podEncoder->GetDistancePerRotation() << std::endl;
    // std::cout << "FPGAIndex: " << m_podEncoder->GetFPGAIndex() << std::endl;
    // std::cout << "GetFreq:   " << m_podEncoder->GetFrequency() << std::endl;
    // std::cout << "GetPosOff: " << m_podEncoder->GetPositionOffset() << std::endl;
    // std::cout << "GetSrcCnl: " << m_podEncoder->GetSourceChannel() << std::endl;
    // std::cout << "Connected: " << m_podEncoder->IsConnected() << std::endl;

    double topMotorSpeed = 0;
    double bottomMotorSpeed = 0;
    double k_maxCommandedSpeed = 0.8;
    double current_angle = ToDegree(m_currentPosition);

    // optimize state (speed, angle) by minimizing change in heading and possibly reversing speed
    auto optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(ToRadian(current_angle)));


    double commanded_speed = (optimizedState.speed.value() / (5200.0 * k_gearRatioYaw));
    
    // std::cout << "Top Vel:    " << m_topEncoder->GetVelocity() << std::endl;
    // std::cout << "Bottom Vel: " << m_bottomEncoder->GetVelocity() << std::endl;

    // std::cout << "CmdSpeed: " << commanded_speed << std::endl;


    /******************************************

    if(xValue || yValue){
        if(0 < xValue){
            topMotorSpeed = -(commanded_speed + factor);
            if(k_maxCommandedSpeed < std::abs(topMotorSpeed)){
                topMotorSpeed = -k_maxCommandedSpeed;
            }
            bottomMotorSpeed = -(commanded_speed - factor);
            if(k_maxCommandedSpeed < std::abs(bottomMotorSpeed)){
                bottomMotorSpeed = -k_maxCommandedSpeed;
            }
        } else {
            topMotorSpeed = commanded_speed + factor;
            if(k_maxCommandedSpeed < std::abs(topMotorSpeed)){
                topMotorSpeed = k_maxCommandedSpeed;
            }
            bottomMotorSpeed = commanded_speed - factor;
            if(k_maxCommandedSpeed < std::abs(bottomMotorSpeed)){
                bottomMotorSpeed = k_maxCommandedSpeed;
            }
        }
    }

**************************************************************/

    if(xValue || yValue){
        double factor = yValue * 0.01;
        if(0 < xValue){
            topMotorSpeed = -(commanded_speed + factor);
            if(k_maxCommandedSpeed < std::abs(topMotorSpeed)){
                topMotorSpeed = -k_maxCommandedSpeed;
            }
            bottomMotorSpeed = -(commanded_speed - factor);
            if(k_maxCommandedSpeed < std::abs(bottomMotorSpeed)){
                bottomMotorSpeed = -k_maxCommandedSpeed;
            }
        } else {
            topMotorSpeed = commanded_speed + factor;
            if(k_maxCommandedSpeed < std::abs(topMotorSpeed)){
                topMotorSpeed = k_maxCommandedSpeed;
            }
            bottomMotorSpeed = commanded_speed - factor;
            if(k_maxCommandedSpeed < std::abs(bottomMotorSpeed)){
                bottomMotorSpeed = k_maxCommandedSpeed;
            }
        }
    }

    m_topMotor->Set(topMotorSpeed);
    m_bottomMotor->Set(bottomMotorSpeed);

//    std::cout <<"X: " << xValue <<  "  Y: " << yValue << "  TopSpeed: " << topMotorSpeed << "  BottomSpeed: " << bottomMotorSpeed << std::endl << std::endl;    
//    std::cout <<"Speed: " << state.speed.value() << "  Optimized: " << optimizedState.speed.value() << std::endl;
}