#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>

class SwervePod: public frc2::SubsystemBase {
    private:
        // The motor that drives the top gear.
        rev::CANSparkMax *m_topMotor;

        // The motor that drives the bottom gear.
        rev::CANSparkMax *m_bottomMotor;

        // The encoder for the wheel position.
        rev::SparkMaxAlternateEncoder *m_encoder;

        frc::Encoder *m_positionEncoder;

        // Desired state -- velocity of wheel in RPM, angle in degrees
        // Angles are measured counter-clockwise, with zero being "robot forward"
        double m_desiredYawDegrees;
        double m_desiredTopMotorSpeed;
        double m_desiredBottomMotorSpeed;

        double m_currentTopMotorSpeed;
        double m_currentBottomMotorSpeed;
        double m_currentAngle;
        
        bool m_initialized;

    public:
        SwervePod(rev::CANSparkMax *topMotor, rev::CANSparkMax *bottomMotor);

        // Initialize this module with the details provided by the robot-specific subclass.
        void Initialize(); 

        // A function to set a direction and speed for this swerve pod
        //angle: -180 - 180, speed: -1.00 - 1.00
        void Drive(double angle, double speed);

        void Periodic() override;
        void SimulationPeriodic() override;
};