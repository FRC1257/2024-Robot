package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotArmIOSim implements PivotArmIO {
    // from here
    // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
    // The P gain for the PID controller that drives this arm.

    // The arm gearbox represents a gearbox containing two Vex 775pro motors.
    private final DCMotor m_armGearbox = DCMotor.getNEO(4);

    // Standard classes for controlling our arm
    private final ProfiledPIDController m_controller;
    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 0);
    private final Encoder m_encoder;

    // Simulation classes help us simulate what's going on, including gravity.
    // This arm sim represents an arm that can travel from -75 degrees (rotated down
    // front)
    // to 255 degrees (rotated down in the back).

    private SingleJointedArmSim sim = new SingleJointedArmSim(
            m_armGearbox,
            PivotArmConstants.PivotArmSimConstants.kArmReduction,
            SingleJointedArmSim.estimateMOI(PivotArmConstants.PivotArmSimConstants.kArmLength, PivotArmConstants.PivotArmSimConstants.kArmMass),
            PivotArmConstants.PivotArmSimConstants.kArmLength,
            PivotArmConstants.PivotArmSimConstants.kMinAngleRads,
            PivotArmConstants.PivotArmSimConstants.kMaxAngleRads,
            true, // change this to true later
            0.1);

    private final EncoderSim m_encoderSim;

    public PivotArmIOSim() {
        m_encoder = new Encoder(PivotArmConstants.PivotArmSimConstants.kEncoderAChannel, PivotArmConstants.PivotArmSimConstants.kEncoderBChannel);
        m_encoderSim = new EncoderSim(m_encoder);
        m_encoderSim.setDistancePerPulse(PivotArmConstants.PivotArmSimConstants.kArmEncoderDistPerPulse);
        m_controller = new ProfiledPIDController(PivotArmConstants.PivotArmSimConstants.kPivotSimPID[0], PivotArmConstants.PivotArmSimConstants.kPivotSimPID[1], PivotArmConstants.PivotArmSimConstants.kPivotSimPID[2],
                new TrapezoidProfile.Constraints(2.45, 2.45));
        
        m_controller.setTolerance(0.1, 0.05);
    }

    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        sim.update(0.02);
        inputs.angleRads = getAngle();
        inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] { sim.getCurrentDrawAmps() };
        inputs.setpointAngleRads = m_controller.getSetpoint().position;
    }

    @Override
    public void setVoltage(double motorVolts) {
        sim.setInputVoltage(motorVolts);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        m_controller.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(getAngle());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput);
    }

    @Override
    public double getAngle() {
        return sim.getAngleRads();
    }

    @Override
    public boolean atSetpoint() {
        return m_controller.atGoal();
    }

    @Override
    public void setP(double p) {
        m_controller.setP(p);
    }

    @Override
    public void setI(double i) {
        m_controller.setI(i);
    }

    @Override
    public void setD(double d) {
        m_controller.setD(d);
    }

    @Override
    public void setkS(double kS) {
        m_feedforward = new SimpleMotorFeedforward(kS, m_feedforward.kv);
    }

    @Override
    public void setkV(double kV) {
        m_feedforward = new SimpleMotorFeedforward(m_feedforward.ks, kV);
    }
    
    @Override
    public double getP() {
        return m_controller.getP();
    }

    @Override
    public double getI() {
        return m_controller.getI();
    }

    @Override
    public double getD() {
        return m_controller.getD();
    }

    @Override
    public double getkS() {
        return m_feedforward.ks;
    }

    @Override
    public double getkV() {
        return m_feedforward.kv;
    }

}
