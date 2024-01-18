//hello
package frc.robot.subsystems.drive;

public class ShooterIOSparkMax implements ShooterIO{
    private CANSparkMax shooterMotor;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    public ShooterIOSparkMax() {
        shooterMotor = new CANSparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setIdleMode(IdleMode.kBrake);
        shooterMotor.setSmartCurrentLimit(NEO_CURRENT_LIMIT);

        encoder = shooterMotor.getEncoder();
        encoder.setPositionConversionFactor(SHOOTER_REV_TO_POS_FACTOR);
        encoder.setVelocityConversionFactor(SHOOTER_REV_TO_POS_FACTOR / 60);
        encoder.setPosition(0.6);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);
    }

     /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = encoder.getPosition();
        inputs.velocityMeters = encoder.getVelocity();
        inputs.appliedVolts = shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage();
        inputs.currentAmps = new double[] {shooterMotor.getOutputCurrent()};
        inputs.tempCelsius = new double[] {shooterMotor.getMotorTemperature()};
    }
}