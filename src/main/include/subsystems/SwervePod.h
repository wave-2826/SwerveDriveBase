#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/PIDSubsystem.h>

/**
 * Swerve Pod encapsulating individual swerve pod functions + attributes
 * including absolute encoder capturing current angle
 * 
 * @author 2826WaveRobotics
 **/
class SwervePod: public frc2::PIDSubsystem {
    private:

        rev::SparkMaxRelativeEncoder *m_topEncoder;
        rev::SparkMaxRelativeEncoder *m_bottomEncoder;

        // The motor that drives the top gear.
        rev::CANSparkMax *m_topMotor;

        // The motor that drives the bottom gear.
        rev::CANSparkMax *m_bottomMotor;

        // The encoder for the wheel position.
        frc::DutyCycleEncoder *m_podEncoder;

        // Desired state -- velocity of wheel in RPM, angle in degrees
        // Angles are measured counter-clockwise, with zero being "robot forward"

        static constexpr const double kP = 0.1;
        static constexpr const double kI = 0.0;
        static constexpr const double kD = 0.0;

        double m_desiredYawDegrees;
        double m_desiredTopMotorSpeed;
        double m_desiredBottomMotorSpeed;

        int m_counter;
        bool m_isReversed = false;

        double m_currentTopMotorSpeed;
        double m_currentBottomMotorSpeed;
        double m_currentPosition;

        double m_previousTopMotorSpeed;
        double m_previousBottomMotorSpeed;
        
        bool m_initialized;

        double turnTuningFactor;
        double offsetAngle;

        double LinearInterpolate(double speed, double targetSpeed, double movePercentage);

    public:
        SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor, double turnTuningFactor, double offsetAngle, int encoderChannel);

        // Initialize this module with the details provided by the robot-specific subclass.
        void Initialize(); 

        /**
         * Function to set a direction and speed for this swerve pod
         * 
         * @param state the Swerve Module state containing speed and angle,
         * with angle: -180 - 180, speed: -1.00 - 1.00
         * @param offsetAngle defines the absolute encoder 0 position in relation to 
         * the "front" or 0 of the robot 
         **/
        void Drive(frc::SwerveModuleState state);

        /**
         * Function that gets the current counter
         * Used as a printout limit for testing/debugging 
         * 
         * @return int value of the cound
         **/ 
        int GetCounter();

        /**
         * Function that sets the current counter
         * Used as a printout limit for testing/debugging 
         * 
         * @param count int value to set the counter to
         **/ 
        void SetCounter(int count);

        /**
         * Function to get swerve pod reversed state, used in optimizing swerve logic to
         * determine if delta angle should be minimized and speed reversed
         **/ 
        bool GetIsReversed();
        /**
         * Function to flip swerve pod reversed state, used in optimizing swerve logic to
         * determine if delta angle should be minimized and speed reversed
         **/ 
        void FlipIsReversed(bool state);

        double GetPreviousTopMotorSpeed();
        void SetPreviousTopMotorSpeed(double value);
        double GetPreviousBottomMotorSpeed();
        void SetPreviousBottomMotorSpeed(double value);

        void Periodic() override;
        void SimulationPeriodic() override;

};