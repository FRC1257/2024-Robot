package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static frc.robot.Constants.PivotArm.PivotArmSimConstants;
import static frc.robot.Constants.PivotArm.PIVOT_ARM_PID;

public class PivotArmIOSim implements PivotArmIO {
    // from here https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
      // The P gain for the PID controller that drives this arm.
  private double m_armSetpointDegrees = PivotArmSimConstants.kDefaultArmSetpointDegrees;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller = new ProfiledPIDController(PIVOT_ARM_PID[0], PIVOT_ARM_PID[1], PIVOT_ARM_PID[2],
  new TrapezoidProfile.Constraints(2.45, 2.45));
  private final Encoder m_encoder =
      new Encoder(PivotArmSimConstants.kEncoderAChannel, PivotArmSimConstants.kEncoderBChannel);
  private final PWMSparkMax motor = new PWMSparkMax(PivotArmSimConstants.kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private SingleJointedArmSim sim = new SingleJointedArmSim(
        m_armGearbox,
        PivotArmSimConstants.kArmReduction,
        SingleJointedArmSim.estimateMOI(PivotArmSimConstants.kArmLength, PivotArmSimConstants.kArmMass),
        PivotArmSimConstants.kArmLength,
        PivotArmSimConstants.kMinAngleRads,
        PivotArmSimConstants.kMaxAngleRads,
        true,
        VecBuilder.fill(PivotArmSimConstants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
    );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    
    public PivotArmIOSim() {
        m_encoderSim.setDistancePerPulse(PivotArmSimConstants.kArmEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        sim.update(0.02);
        inputs.angle = getAngle();
        inputs.angleRadsPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
        
        /* m_encoderSim.setDistance(sim.getPositionMeters());
        inputs.positionMeters = sim.getPositionMeters();
        inputs.velocityMeters = sim.getVelocityMetersPerSecond();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()}; */
    }

    @Override
    public void setVoltage(double motorVolts) {
        sim.setInputVoltage(motorVolts);
    }

    @Override
    public void goToSetpoint(double setpoint) {
        m_controller.setGoal(setpoint);
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(m_encoder.getDistance());
        double feedforwardOutput = 0; // m_feedforward.calculate(m_controller.getSetpoint().velocity);

        sim.setInputVoltage(feedforwardOutput + pidOutput);
    }

    @Override
    public double getAngle() {
        return Units.radiansToDegrees(sim.getAngleRads()) - 36;
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
    public void setFF(double ff) {

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
    
}
