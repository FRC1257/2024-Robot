// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static enum Drivers {
    MAUI,
    PROGRAMMERS
  }

  public static enum Operators {
    ERICK,
    PROGRAMMERS
  }

  public static final Drivers driver = Drivers.MAUI;
  public static final Operators operator = Operators.ERICK;

  public static final boolean tuningMode = true;

  

  

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class ElectricalLayout {
    // Controllers
    public final static int CONTROLLER_DRIVER_ID = 0;
    public final static int CONTROLLER_OPERATOR_ID = 1;

    // PLACEHOLDER Intake
    public final static int INTAKE_MOTOR = 15;
    public final static int GROUND_INTAKE_MOTOR = 9;

    // Intake Sensors
    public final static int INTAKE_PHOTO_ELECTRIC = 0;
    public final static int GROUND_INTAKE_BREAK_BEAM = 1;
    
    // CHANGE CONSTANTS, THESE ARE TEMPORARY
    public static final int PIVOT_ARM_ID = 16;
    public static final int LEFT_SLAVE_ID = 12;
    public static final int RIGHT_SLAVE_FRONT_ID = 10;
    public static final int RIGHT_SLAVE_BACK_ID = 11;
    
    public static final int INTAKE_BREAK_BEAM = 5;

    public static final int ABSOLUTE_ENCODER_ID = 1;

    // LED
    public static final int BLINKIN_LED_CONTROLLER_PORT = 1;

    // Shooter
    public static final int SHOOTER_LEFT_ID = 14; // master
    public static final int SHOOTER_RIGHT_ID = 13;
  };

  public final static int NEO_CURRENT_LIMIT = 80; // amps
  public final static int NEO_VORTEX_CURRENT_LIMIT = 60;

  public static double PI = 3.141592653589793238462643;
  public static final double[][] LookupTable = {
    {0,1000,10},
    {1,2000,30},
    {2,3000,50},
    {3,4000,60},
    {4,5500,70},
    {5,6500,70},
    {6,6500,75},
    {7,6500,77},
    {8,6700,79},
    {9,6900,83},
    {10,6500,85}
  };

}

