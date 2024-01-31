package frc.robot.subsystems.pivotArm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.pivotArm.PivotArm;

import static frc.robot.Constants.PivotArm.PivotArmSimConstants.*;

public class PivotArmIOSim implements PivotArmIO {
    // from here https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armsimulation/subsystems/Arm.java
      // The P gain for the PID controller that drives this arm.
  private double m_armSetpointDegrees = kDefaultArmSetpointDegrees;
    
  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_controller;
  private final Encoder m_encoder;
  private final PWMSparkMax motor = new PWMSparkMax(kMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).

  private SingleJointedArmSim sim = new SingleJointedArmSim(
    m_armGearbox,
    kArmReduction,
    SingleJointedArmSim.estimateMOI(kArmLength, kArmMass),
    kArmLength,
    kMinAngleRads,
    kMaxAngleRads,
    false,
    0
  );

  private final EncoderSim m_encoderSim;
    
    public PivotArmIOSim() {
      m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
      m_encoderSim = new EncoderSim(m_encoder);
      m_encoderSim.setDistancePerPulse(kArmEncoderDistPerPulse);
      m_controller = new ProfiledPIDController(kPivotSimPID[0], kPivotSimPID[1], kPivotSimPID[2],
  new TrapezoidProfile.Constraints(2.45, 2.45));
    }

    @Override
    public void updateInputs(PivotArmIOInputs inputs) {
        sim.update(0.02);
        inputs.angleRads = getAngle();
        inputs.angVelocityRadsPerSec = sim.getVelocityRadPerSec();
        inputs.currentAmps = new double[] {sim.getCurrentDrawAmps()};
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
