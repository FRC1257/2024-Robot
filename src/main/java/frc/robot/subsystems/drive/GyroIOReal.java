package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper class for the navX-MXP and ADXRS450_Gyro
 * 
 * Mostly utilizes the navX-MXP, but uses the ADXRS450 as backup if it disconnects
 */
public class GyroIOReal {
    // this class is not like the other Advantage Kit classes because we only want it
    // to run on the real robot
    // we can log all this stuff else where if we want to test it
    private static GyroIOReal instance = new GyroIOReal();

    private final AHRS navx;
    private double resetRoll;
    private double resetPitch;
    private double resetYaw;

    private final ADXRS450_Gyro gyro;

    private GyroIOReal() {
        navx = new AHRS();
        resetRoll = 0;
        resetPitch = 0;
        navx.reset();

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
    }

    /**
     * Gets the current yaw angle.
     * 
     * @return The angle in degrees limited to the range -180 to 180.
     */
    public double getYawAngle() {
        double angle = navx.getAngle() - resetYaw;
        while(angle <= -180) angle += 360;
        while(angle > 180) angle -= 360;

        return angle;
    }

    /**
     * Gets the current yaw angle velocity in deg / s
     * 
     * @return The yaw angle rate in degrees.
     */
    public double getYawAngleVelocity() {
        return navx.getRate();
    }

    /**
     * Gets the current roll angle.
     * 
     * @return The angle in degrees.
     */
    public double getRollAngle() {
        return navx.getRoll() - resetRoll;
    }

    /**
     * Gets the current pitch angle.
     * 
     * @return The angle in degrees.
     */
    public double getPitchAngle() {
        return navx.getPitch() - resetPitch;
    }

    /**
     * Sets the current yaw angle to "0".
     */
    public void zeroYawAngle() {
        // navx.setAngleAdjustment(-navx.getAngle());
        resetYaw = navx.getAngle();
    }

    /**
     * Sets the current yaw angle to angle.
     */
    public void setYawAngle(double angle) {
        // navx.setAngleAdjustment(angle - navx.getAngle());
        resetYaw = navx.getAngle() - angle;
    }

    public void setRollAngle(double angle) {
        resetRoll = navx.getAngle() - angle;
    }

    /**
     * Sets the current roll angle to "0".
     */
    public void zeroRollAngle() {
        resetRoll = getRollAngle();
    }

    /**
     * Sets the current pitch angle to "0".
     */
    public void zeroPitchAngle() {
        resetPitch = getPitchAngle();
    }

    /**
     * Gets the current rotation of the robot.
     * 
     * @return The angle in degrees.
     */
    public double getRobotAngle() {
        return getYawAngle();
    }

    /**
     * Gets the current rotation rate of the robot in deg/s
     * 
     * @return The angle rate in degrees.
     */
    public double getRobotAngleVelocity() {
        return getYawAngleVelocity();
    }

    /**
     * Gets the current tilt of the robot. Utilizes the NavX if it is connected, but
     * otherwise it will use the backup gyro
     * 
     * @return THe angle in degrees
     */
    public double getTiltAngle() {
        if (navXConnected()) {
            return getPitchAngle();
        } else {
            return -gyro.getAngle();
        }
    }

    /**
     * Sets the current rotation of the robot to "0".
     */
    public void zeroRobotAngle() {
        zeroYawAngle();
    }

    /**
     * Sets the current rotation of the robot to a given value.
     */
    public void setRobotAngle(double angle) {
        setYawAngle(angle);
    }

    /**
     * Sets the current tilt of the robot to "0".
     */
    public void zeroTiltAngle() {
        zeroPitchAngle();
        gyro.reset();
    }

    public void zeroAll() {
        zeroYawAngle();
        zeroPitchAngle();
        zeroRollAngle();
    }

    /**
     * Returns whether or not the NavX is currently connected and sending valid data
     * 
     * @return whether or not the NavX is currently connected
     */
    public boolean navXConnected() {
        return navx.isConnected();
    }

    /**
     * Displays the angles on {@code SmartDashboard}.
     */
    public void outputValues() {
        SmartDashboard.putNumber("/Gyro/Yaw Angle", getYawAngle());
        SmartDashboard.putNumber("/Gyro/Roll Angle", getRollAngle());
        SmartDashboard.putNumber("/Gyro/Pitch Angle", getPitchAngle());

        SmartDashboard.putNumber("Robot Angle", getRobotAngle());
        SmartDashboard.putNumber("Robot Angle Vel", getRobotAngleVelocity());
        // SmartDashboard.putNumber("Tilt Angle", getTiltAngle());
        SmartDashboard.putBoolean("Gyro Connected", navXConnected());
    }

    public static GyroIOReal getInstance() {
        if (instance == null) {
            instance = new GyroIOReal();
        }
        return instance;
    }

    public ADXRS450_Gyro getGyro() {
        return gyro;
    }
}