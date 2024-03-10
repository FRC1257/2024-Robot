package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectricalLayout;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ElectricalLayout.*;

public class Pivot extends SubsystemBase {
    private double setpoint = 0;

    private CANSparkMax pivotMotor, leftSlave, rightSlaveFront, rightSlaveBack;
    private final ProfiledPIDController pidController;
    private ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

        private DutyCycleEncoder absoluteEncoder;
    private RelativeEncoder motorEncoder;

    // Create a Mechanism2d visualization of the arm
    private MechanismLigament2d armMechanism = getArmMechanism();

    public Pivot() {
        pivotMotor = new CANSparkMax(PIVOT_ARM_ID, MotorType.kBrushless);
        leftSlave = new CANSparkMax(LEFT_SLAVE_ID, MotorType.kBrushless);
        rightSlaveFront = new CANSparkMax(RIGHT_SLAVE_FRONT_ID, MotorType.kBrushless);
        rightSlaveBack = new CANSparkMax(RIGHT_SLAVE_BACK_ID, MotorType.kBrushless);

        pivotMotor.restoreFactoryDefaults();
        leftSlave.restoreFactoryDefaults();
        rightSlaveFront.restoreFactoryDefaults();
        rightSlaveBack.restoreFactoryDefaults();
        //I swear to go these inverts might have to be after the following
        //pivotMotor.setInverted(false);
        //leftSlave.setInverted(false);
        //rightSlaveFront.setInverted(true);
        //rightSlaveBack.setInverted(true);
        //can't invert like this
        setBrake(true);

        leftSlave.follow(pivotMotor, false);
        rightSlaveFront.follow(pivotMotor, true);
        rightSlaveBack.follow(pivotMotor, true);

        setBrake(true);
        
        pivotMotor.enableVoltageCompensation(12.0);

        pivotMotor.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        leftSlave.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        rightSlaveFront.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);
        rightSlaveBack.setSmartCurrentLimit(Constants.NEO_CURRENT_LIMIT);

        pivotMotor.burnFlash();
        leftSlave.burnFlash();
        rightSlaveFront.burnFlash();
        rightSlaveBack.burnFlash();

        //wasn't burning the flash to all the motors, this might be the issue

        absoluteEncoder = new DutyCycleEncoder(ElectricalLayout.ABSOLUTE_ENCODER_ID);
        absoluteEncoder.setDistancePerRotation(2 * Constants.PI * PivotArmConstants.POSITION_CONVERSION_FACTOR);
        absoluteEncoder.setDutyCycleRange(1/1024.0, 1023.0/1024.0);
        // make sure the pivot starts at the bottom position every time
        // absoluteEncoder.reset();

        pidController = new ProfiledPIDController(PivotArmConstants.PIVOT_ARM_PID_REAL[0], PivotArmConstants.PIVOT_ARM_PID_REAL[1], PivotArmConstants.PIVOT_ARM_PID_REAL[2],
                new TrapezoidProfile.Constraints(2.45, 2.45));
        
        pidController.setTolerance(PivotArmConstants.PIVOT_ARM_PID_TOLERANCE, PivotArmConstants.PIVOT_ARM_PID_VELOCITY_TOLERANCE);

 
        //0 position for absolute encoder is at 0.2585 rad, so subtract that value from everything

        motorEncoder = pivotMotor.getEncoder();
        motorEncoder.setPositionConversionFactor(PivotArmConstants.POSITION_CONVERSION_FACTOR);
        motorEncoder.setVelocityConversionFactor(PivotArmConstants.POSITION_CONVERSION_FACTOR / 60.0);
        configurePID();
        configureFeedForward();
    }

    private void configurePID() {
        // pidController.setOutputRange(PivotArmConstants.PIVOT_ARM_MIN_ANGLE, PivotArmConstants.PIVOT_ARM_MAX_ANGLE);
        pidController.setP(PivotArmConstants.PIVOT_ARM_PID_REAL[0]);
        pidController.setI(PivotArmConstants.PIVOT_ARM_PID_REAL[1]);
        pidController.setD(PivotArmConstants.PIVOT_ARM_PID_REAL[2]);
    }

    private void configureFeedForward() {
        setkS(PivotArmConstants.PIVOT_ARM_FEEDFORWARD_REAL[0]);
        setkG(PivotArmConstants.PIVOT_ARM_FEEDFORWARD_REAL[1]);
        setkV(PivotArmConstants.PIVOT_ARM_FEEDFORWARD_REAL[2]);
        setkA(PivotArmConstants.PIVOT_ARM_FEEDFORWARD_REAL[3]);
    }
    
    public void periodic() {
        
    }

    public void setBrake(boolean brake) {
        pivotMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        leftSlave.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        rightSlaveFront.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        rightSlaveBack.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public double getAngle() {
        return -absoluteEncoder.getDistance() + PivotArmConstants.PIVOT_ARM_OFFSET;
    }

    public void setVoltage(double motorVolts) {
        // limit the arm if its past the limit
        if (getAngle() > PivotArmConstants.PIVOT_ARM_MAX_ANGLE && motorVolts > 0) {
            motorVolts = 0;
        } else if (getAngle() < PivotArmConstants.PIVOT_ARM_MIN_ANGLE && motorVolts < 0) {
            motorVolts = 0;
        }
        
        pivotMotor.setVoltage(motorVolts);
    }

    public void move(double speed) {
        setVoltage(speed);
    }

    public void runPID() {
        goToSetpoint(setpoint);
    }

    public void setPID(double setpoint) {
        this.setpoint = setpoint;
    }

    public void addPID(double setpointAdd) {
        this.setpoint += setpointAdd;
        this.setpoint = MathUtil.clamp(this.setpoint, PivotArmConstants.PIVOT_ARM_MIN_ANGLE, PivotArmConstants.PIVOT_ARM_MAX_ANGLE);
        
    }

    public boolean atSetpoint() {
        return Math.abs(getAngle() - setpoint) < PivotArmConstants.PIVOT_ARM_PID_TOLERANCE;
    }

    public void setMechanism(MechanismLigament2d mechanism) {
        armMechanism = mechanism;
    }

    public MechanismLigament2d append(MechanismLigament2d mechanism) {
        return armMechanism.append(mechanism);
    }

    public MechanismLigament2d getArmMechanism() {
        return new MechanismLigament2d("Pivot Arm", 0.4, 0, 5, new Color8Bit(Color.kAqua));
    }

    public Command PIDCommand(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }

    public Command PIDCommandForever(DoubleSupplier setpointSupplier) {
        return new FunctionalCommand(
            () -> setPID(setpointSupplier.getAsDouble()), 
            () -> {
                setPID(setpointSupplier.getAsDouble());
                runPID();
            }, 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command PIDCommandForever(double setpoint) {
        return new FunctionalCommand(
            () -> setPID(setpoint), 
            () -> runPID(), 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command PIDHoldCommand() {
        return new FunctionalCommand(
            () -> setPID(getAngle()), 
            () -> runPID(), 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public Command PIDCommand(DoubleSupplier setpointSupplier) {
        return new FunctionalCommand(
            () -> setPID(setpointSupplier.getAsDouble()), 
            () -> {
                setPID(setpointSupplier.getAsDouble());
                runPID();
            }, 
            (stop) -> move(0), 
            this::atSetpoint, 
            this
        );
    }

    // Allows manual control of the pivot arm for PID tuning
    public Command ManualCommand(DoubleSupplier speedSupplier) {
        return new FunctionalCommand(
            () -> move(speedSupplier.getAsDouble()), 
            () -> move(speedSupplier.getAsDouble()), 
            (stop) -> move(0), 
            () -> false, 
            this
        );
    }

    public void stop() {
        // return new InstantCommand(
        //     () -> move(0), 
        //     this
        // );
        move(0);
    }//not calling move
    //no commmand yalee

    /** Go to Setpoint */
    public void goToSetpoint(double setpoint) {
        pidController.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = MathUtil.clamp(pidController.calculate(getAngle()), -3, 3);
        double feedforwardOutput = feedforward.calculate(getAngle(), pidController.getSetpoint().velocity);

        setVoltage(MathUtil.clamp(pidOutput + feedforwardOutput, -4, 4));
    }

    public void setP(double p) {
        pidController.setP(p);
    }

    public void setI(double i) {
        pidController.setI(i);
    }

    public void setD(double d) {
        pidController.setD(d);
    }

    public void setFF(double ff) {
        // pidController.setFF(ff);
    }

    public void setkS(double kS) {
        feedforward = new ArmFeedforward(kS, feedforward.kg, feedforward.kv, feedforward.ka);
    }

    public void setkG(double kG) {
        feedforward = new ArmFeedforward(feedforward.ks, kG, feedforward.kv, feedforward.ka);
    }

    public void setkV(double kV) {
        feedforward = new ArmFeedforward(feedforward.ks, feedforward.kg, kV, feedforward.ka);
    }

    public void setkA(double kA) {
        feedforward = new ArmFeedforward(feedforward.ks, feedforward.kg, feedforward.kv, kA);
    }
 
    public double getkS(){
        return feedforward.ks;
    }
 
    public double getkG(){
        return feedforward.kg;
    }
 
    public double getkV(){
        return feedforward.kv;
    }
 
    public double getkA(){
        return feedforward.ka;
    }

    public double getP() {
        return pidController.getP();
    }

    public double getI() {
        return pidController.getI();
    }

    public double getD() {
        return pidController.getD();
    }

    public double getFF() {
        // return pidController.getFF();
        return 0;
    }

}

