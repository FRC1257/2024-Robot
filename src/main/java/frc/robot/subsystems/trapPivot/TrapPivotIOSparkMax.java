package frc.robot.subsystems.trapPivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import static frc.robot.Constants.ElectricalLayout.*;
import static frc.robot.Constants.TrapPivot.*;
import static frc.robot.Constants.TrapPivot.TrapPivotReal.*;

public class TrapPivotIOSparkMax implements TrapPivotIO {
    private CANSparkMax pivotMotor;
    private SparkPIDController pidController;
    private RelativeEncoder encoder;
    private DutyCycleEncoder absoluteEncoder;

    public TrapPivotIOSparkMax() {
        pivotMotor = new CANSparkMax(TRAP_PIVOT_ID, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setInverted(false);

        pivotMotor.enableVoltageCompensation(12.0);
        pivotMotor.setSmartCurrentLimit(30);
        pivotMotor.burnFlash();

        configureEncoders();
        configurePID();
    }

    private void configureEncoders() {
        encoder = pivotMotor.getEncoder();

        encoder.setPositionConversionFactor(Math.PI * TRAP_PIVOT_ROTATION_DIAM_M / TRAP_PIVOT_GEARBOX_REDUCTION);
        encoder.setVelocityConversionFactor(Math.PI * TRAP_PIVOT_ROTATION_DIAM_M / TRAP_PIVOT_GEARBOX_REDUCTION / 60.0);

        absoluteEncoder = new DutyCycleEncoder(0);
        absoluteEncoder.setDistancePerRotation(360.0 / 1024.0);
        absoluteEncoder.setDutyCycleRange(1 / 1024.0, 1023.0 / 1024.0);

        encoder.setPosition(absoluteEncoder.getDistance() * 28.45 + 0.6);
    }

    private void configurePID() {
        pidController = pivotMotor.getPIDController();
        pidController.setP(TRAP_PIVOT_PID_REAL[0]);
        pidController.setI(TRAP_PIVOT_PID_REAL[1]);
        pidController.setD(TRAP_PIVOT_PID_REAL[2]);
    }
}
