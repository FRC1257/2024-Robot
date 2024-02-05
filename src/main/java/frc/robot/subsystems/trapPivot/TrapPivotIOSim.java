package frc.robot.subsystems.trapPivot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static frc.robot.Constants.TrapPivot.TrapPivotSim.*;

public class TrapPivotIOSim implements TrapPivotIO {
    // from here https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
    // The P gain for the PID controller that drives this arm.
    private double armSetpointDegrees = DEFAULT_ARM_SETPOINT_DEGREES;
    
    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor gearbox = DCMotor.getVex775Pro(2);

    // Standard classes for controlling our arm
    private final ProfiledPIDController pidController;
    private final Encoder encoder;
    private final PWMSparkMax motor = new PWMSparkMax(TRAP_PIVOT_ID);

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
    // to 255 degrees (rotated down in the back).
    private SingleJointedArmSim sim;
    private EncoderSim encoderSim;

    public TrapPivotIOSim() {
        sim = new SingleJointedArmSim(
            gearbox,
            TRAP_ARM_REDUCTION,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_M, ARM_MASS_KG),
            ARM_LENGTH_M,
            MIN_ANGLE_RADS,
            MAX_ANGLE_RADS,
            false,
            0
        );
        encoder = new Encoder(ENCODER_A_CHANNEL, ENCODER_B_CHANNEL);
        encoderSim = new EncoderSim(encoder);
        encoderSim.setDistancePerPulse(DISTANCE_PER_PULSE);
        pidController = new ProfiledPIDController(TRAP_PIVOT_PID_SIM[0], TRAP_PIVOT_PID_SIM[1], TRAP_PIVOT_PID_SIM[2],
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    }

    @Override
    public void updateInputs(TrapPivotIOInputs inputs) {
        sim.update(0.02);
        inputs.angleRads = sim.getAngleRads();
        inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        pidController.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = pidController.calculate(encoder.getDistance());
        double feedForwardOutput = 0;

        sim.setInputVoltage(feedForwardOutput + pidOutput);
    }
}
