#include "subsystems/SwervePod.h"
#include "math.h"
#include <iostream>
#include <string>


SwervePod::SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor, double turnTuningFactor, double angleOffset, int encoderChannel) 
{
    m_topMotor = topMotor;
    m_bottomMotor = bottomMotor;
    m_topEncoder = new rev::SparkMaxRelativeEncoder(m_topMotor->GetEncoder());
    m_bottomEncoder = new rev::SparkMaxRelativeEncoder(m_bottomMotor->GetEncoder());

    turnTuningFactor = turnTuningFactor;
    angleOffset = angleOffset;

    // DIO - returns val as a ratio of hi-time v low-time
    m_podEncoder = new frc::DutyCycleEncoder(encoderChannel);
    m_podEncoder->SetConnectedFrequencyThreshold(975.6);
}

// TODO: Move to a conversion/util file
// Convert from absolute encoder position to degrees
double ToDegree(double pos)
{
    // double deg_value = pos/1024.0*360.0;
    double deg_value = pos * 360.0;

    return deg_value;
}

// TODO: Move to a conversion/util file
// Convert degrees to radians
double ToRadian(double degrees)
{
    return degrees * (3.14159 / 180.0);
}

void SwervePod::Periodic()
{
    // Put code here to be run every loop
}

void SwervePod::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
}

void SwervePod::Initialize()
{
    // 0 is mechanical - offsets will have to be set in code
    m_initialized = true;
    m_counter = 0;
}

int SwervePod::GetCounter()
{
    return m_counter;
}

void SwervePod::SetCounter(int count)
{
    m_counter = count;
}

bool SwervePod::GetIsReversed()
{
    return m_isReversed;
}

void SwervePod::FlipIsReversed(bool state)
{
    if (state == true)
    {
        m_isReversed = false;
    }
    else
    {
        m_isReversed = true;
    }
}

double SwervePod::LinearInterpolate(double speed, double targetSpeed, double movePercentage) {
    double newSpeed = speed;
    // current speed is less than target speed
    if (speed < targetSpeed)
    {
        newSpeed = speed + std::fabs(targetSpeed - speed) * movePercentage;
    }
    // current speed is greater than target speed
    else if (speed > targetSpeed)
    {
        newSpeed = speed - std::fabs(targetSpeed - speed) * movePercentage;
    }
    // adding a buffer between newSpeed and targetSpeed
    if (std::abs(targetSpeed - newSpeed) < 0.01f) {
        newSpeed = targetSpeed;
    }
    // prevent newSpeed from going outside of physical boundaries
    if (newSpeed <= -1.0f)
    {
        newSpeed = -1.0f;
    }
    else if (newSpeed >= 1.0f)
    {
        newSpeed = 1.0f;
    }

    return newSpeed;
}

void SwervePod::SetPreviousTopMotorSpeed(double value) {
    m_previousTopMotorSpeed = value;
}

double SwervePod::GetPreviousTopMotorSpeed() {
    return m_previousTopMotorSpeed;
}

void SwervePod::SetPreviousBottomMotorSpeed(double value) {
    m_previousBottomMotorSpeed = value;
}

double SwervePod::GetPreviousBottomMotorSpeed() {
    return m_previousBottomMotorSpeed;
}

void SwervePod::Drive(frc::SwerveModuleState state)
{
    // TODO: set max motor speeds
    double topMotorSpeed = 0;
    double bottomMotorSpeed = 0;

    // tracking angles
    // pod state angle (target) is -180 to 180
    // encoder angle (current) is 0 - 360
    // offsetAngle adjusts "forward" to align to position on robot
    double current_angle = offsetAngle + ToDegree(m_podEncoder->GetAbsolutePosition());
    double target_angle;
    if (GetIsReversed()) {
        target_angle = state.angle.Degrees().value() + 180;

    } else {
        target_angle = state.angle.Degrees().value() + 360;
        if (target_angle >= 360) {
            target_angle -= 360;
        }
    }

    // optimize state (speed, angle) by minimizing change in heading and possibly reversing speed
    // auto optimizedState = frc::SwerveModuleState::Optimize(state, units::radian_t(ToRadian(current_angle)));
    
    // conversion from input state speed (rpm) to a motor power % val (0-1)
    double commanded_speed = (state.speed.value() / 5200.0);
    if (1 < commanded_speed)
    {
        commanded_speed = 1;
    }

    double angle_delta = target_angle - current_angle;
    if (angle_delta > 180.0)
    {
        angle_delta -= 360.0;
    }
    else if (angle_delta < -180.00)
    {
        angle_delta += 360.0;
    }
    double angle_delta_optimized = 0.0;
    double normalizer = 1 / 180.0;
    std::string swerveCase = "DID NOT ENTER";

    if (fabs(angle_delta) < 45.0)
    {
        double tunedAngleDelta = angle_delta * turnTuningFactor;
        // check aligned
        swerveCase = "ALIGNED";

        // station keep alg
        double stationKeepTop;
        double stationKeepBottom;
        double divisor;
        if (GetIsReversed())
        {
            stationKeepTop = (1 - tunedAngleDelta * normalizer);
            stationKeepBottom = (1 + tunedAngleDelta * normalizer);
        }
        else
        {
            stationKeepTop = (1 + tunedAngleDelta * normalizer);
            stationKeepBottom = (1 - tunedAngleDelta * normalizer);
        }
        if (fabs(stationKeepTop) > fabs(stationKeepBottom))
        {
            divisor = stationKeepTop / stationKeepBottom;
        }
        else
        {
            divisor = stationKeepBottom / stationKeepTop;
        }

        if (GetIsReversed())
        {
            topMotorSpeed = -commanded_speed * (stationKeepTop / divisor);
            bottomMotorSpeed = -commanded_speed * (stationKeepBottom / divisor);
        }
        else
        {
            topMotorSpeed = commanded_speed * (stationKeepTop / divisor);
            bottomMotorSpeed = commanded_speed * (stationKeepBottom / divisor);
        }
    }
    else if (angle_delta < -90.0)
    {
        // check optimal path
        swerveCase = "OPTIMIZE < -90";

        FlipIsReversed(m_isReversed);
        if (!GetIsReversed())
        {
            angle_delta_optimized = (angle_delta - 180.0);
        }
        else
        {
            angle_delta_optimized = (angle_delta + 180.0);
        }

        topMotorSpeed = -commanded_speed * angle_delta_optimized * normalizer;
        bottomMotorSpeed = commanded_speed * angle_delta_optimized * normalizer;
    }
    else if (angle_delta > 90.0)
    {
        // check optimal path
        swerveCase = "OPTIMIZE > 90";

        FlipIsReversed(m_isReversed);
        if (!GetIsReversed())
        {
            angle_delta_optimized = (angle_delta - 180.0);
        }
        else
        {
            angle_delta_optimized = (angle_delta + 180.0);
        }

        topMotorSpeed = commanded_speed * angle_delta_optimized * normalizer;
        bottomMotorSpeed = -commanded_speed * angle_delta_optimized * normalizer;
    }
    else
    {
        swerveCase = "ROTATE";

        topMotorSpeed = commanded_speed * angle_delta * normalizer;
        bottomMotorSpeed = -commanded_speed * angle_delta * normalizer;
    }

    // assign motor speeds
    // TODO: interpolate motor speeds / slowly ramp up + down
    // double interpolatedTopMotorSpeed = LinearInterpolate(GetPreviousTopMotorSpeed(), topMotorSpeed, 0.15);
    // double interpolatedBottomMotorSpeed = LinearInterpolate(GetPreviousBottomMotorSpeed(), bottomMotorSpeed, 0.15);
    // m_topMotor->Set(interpolatedTopMotorSpeed);
    // m_bottomMotor->Set(interpolatedBottomMotorSpeed);
    // SetPreviousTopMotorSpeed(interpolatedTopMotorSpeed);
    // SetPreviousBottomMotorSpeed(interpolatedBottomMotorSpeed);

    m_topMotor->Set(topMotorSpeed);
    m_bottomMotor->Set(bottomMotorSpeed);

    ///////////////////////////////// TESTING PRINTOUTS ///////////////////////////////////////////////

    // if (GetCounter() > 5)
    // {

    //     std::cout << "Top:    " << interpolatedTopMotorSpeed << std::endl;
    //     std::cout << "Bottom: " << interpolatedBottomMotorSpeed << std::endl;
    //     std::cout << std::endl;
    //     std::cout << "angle " << current_angle << std::endl;
    //     std::cout << "angle delta " << angle_delta << std::endl;
    //     // std::cout << std::endl;
    //     std::cout << swerveCase << std::endl;
    //     // std::cout << "top motor: " << topMotorSpeed << std::endl;
    //     // std::cout << "bottom motor: " << bottomMotorSpeed << std::endl;

    //     // std::cout << "commanded speed: " << commanded_speed << std::endl;
    //     // std::cout << "tuning value: " << normalizer << std::endl;
    //     // commanded_speed * angle_delta_optimized * normalizer

    //     // std::cout << std::endl << "COMMANDED SPEED: " << commanded_speed << std::endl << std::endl;

    //     // std::cout << std::endl;

    //     // std::cout << "angle delta: " << angle_delta << std::endl;
    //     // std::cout << "current angle: " << current_angle << std::endl;
    //     // std::cout << "target angle: " << target_angle << std::endl;
    //     // std::cout << "optimized angle delta: " << angle_delta_optimized << std::endl;

    //     // std::cout << "angle delta: " << angle_delta << std::endl;
    //     // std::cout << "tuning value : " << normalizer << std::endl;
    //     // std::cout << "kinematics optimized target angle: " << optimizedState.angle.Degrees().value() << std::endl;

    //     // std::cout << "Degrees: " << ToDegree(m_podEncoder->GetAbsolutePosition()) << std::endl;
    //     // std::cout << "Radian: " << ToRadian(ToDegree(m_podEncoder->GetAbsolutePosition())) << std::endl;

    //     // std::cout << "State Speed: " << state.speed.value() << std::endl;
    //     // std::cout << "State Angle: " << state.angle.Degrees().value() << std::endl;
    //     SetCounter(0);
    // }
    // else
    // {
    //     int current_count = GetCounter();
    //     SetCounter(current_count + 1);
    // }
}
