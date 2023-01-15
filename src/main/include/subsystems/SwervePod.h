#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/kinematics/SwerveModuleState.h>

enum PodMode {
    Pod_Off,
    Pod_Move,
    Pod_Rotate,
    Pod_RotateAndMove
};

class SwervePod: public frc2::SubsystemBase {
    private:

        rev::SparkMaxRelativeEncoder *m_topEncoder;
        rev::SparkMaxRelativeEncoder *m_bottomEncoder;

        // The motor that drives the top gear.
        rev::CANSparkMax *m_topMotor;

        // The motor that drives the bottom gear.
        rev::CANSparkMax *m_bottomMotor;

        // The encoder for the Motor
        //rev::SparkMaxAlternateEncoder *m_encoder;

        // The encoder for the wheel position.
        frc::DutyCycleEncoder *m_podEncoder;
    

        // Desired state -- velocity of wheel in RPM, angle in degrees
        // Angles are measured counter-clockwise, with zero being "robot forward"
        double m_desiredYawDegrees;
        double m_desiredTopMotorSpeed;
        double m_desiredBottomMotorSpeed;

        int m_counter;
        bool m_isReversed;

        double m_currentTopMotorSpeed;
        double m_currentBottomMotorSpeed;
        double m_currentPosition;
        
        PodMode m_podOperationMode;
        bool m_initialized;

    public:
        SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor, int encoderChannel);

        // Initialize this module with the details provided by the robot-specific subclass.
        void Initialize(); 

        // A function to set a direction and speed for this swerve pod
        //angle: -180 - 180, speed: -1.00 - 1.00
        void Drive(frc::SwerveModuleState state, double xValue,double yValue);

        int GetCounter();
        void SetCounter(int count);

        bool GetIsReversed();
        void FlipIsReversed(bool state);

        //void Drive(frc::SwerveModuleState state, double xValue, double yValue);

        void Periodic() override;
        void SimulationPeriodic() override;

        // Gear ratio for yaw. This may not include all pairs of gears.
        const double k_gearRatioYaw = 5.3125;
        // Gear ratio for wheel speed. Typically, this includes all pairs of gears.
        const double k_gearRatioWheelSpeed = 3.2196;
        // Maximum yaw speed in RPM
        const double k_maxYawSpeedRPM = 0.0;
        // Wheel diameter in meters
        const double k_wheelDiameterMeters = 0.0635;
        // Wheel circumference in meters
        const double k_wheelCircumferenceMeters = k_wheelDiameterMeters * (double)3.141592653;
        // Max motor speed
        const double k_maxMotorSpeed = 5200.0;
};