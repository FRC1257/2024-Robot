// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.pivotArm.PivotArm;
import frc.robot.subsystems.pivotArm.PivotArmConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode mode = Mode.REAL;
  public static final Drivers driver = Drivers.MAUI;
  public static final Operators operator = Operators.ERICK;

  public static final Mode currentMode = getRobotMode();

  //public static final Mode currentMode = Mode.SIM;


  public static final boolean tuning = true;

  public static final boolean tuningMode = true;
  public static final boolean useVision = true;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /* Test bot */
    TEST,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Drivers {
    MAUI,
    PROGRAMMERS
  }

  public static enum Operators {
    ERICK,
    PROGRAMMERS
  }

  public static Mode getRobotMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    }
    if (RobotBase.isSimulation()) {
      switch (mode) {
        case REAL:
          System.out.println("WARNING: Running in real mode while in simulation");
        case SIM:
          return Mode.SIM;
        case TEST:
          return Mode.TEST;
        case REPLAY:
          return Mode.REPLAY;
      }
    }
    return Mode.REAL;
  }

  public static class BuildConstants {
    public static int DIRTY = 1;
    public static String MAVEN_NAME = "Snail";
    public static String BUILD_DATE = "12/57";
    public static String GIT_SHA = "Snail";
    public static String GIT_DATE = "Snail";
    public static String GIT_BRANCH = "PivotArm";
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
    public final static int INTAKE_PHOTO_ELECTRIC = 9;
    
    public static final int PIVOT_ARM_ID = 16;
    public static final int LEFT_SLAVE_ID = 12;
    public static final int RIGHT_SLAVE_FRONT_ID = 10;
    public static final int RIGHT_SLAVE_BACK_ID = 11;

    public static final int ABSOLUTE_ENCODER_ID = 8;

    // LED
    public static final int BLINKIN_LED_CONTROLLER_PORT = 1;

    // Shooter
    public static final int SHOOTER_LEFT_ID = 14; // master
    public static final int SHOOTER_RIGHT_ID = 13;
    public static final int PHOTOELECTRIC_SENSOR_CHANNEL = 17; // NEEDS TO BE CHANGED 
  };

  public static double PI = 3.141592653589793238462643;
  public static double UPDATE_PERIOD = 0.010; // seconds
  public final static int NEO_550_CURRENT_LIMIT = 25; // amps
  public final static int NEO_VORTEX_CURRENT_LIMIT = 60;
  public final static int QUADRATURE_COUNTS_PER_REV = 8192; // encoder resolution
  // https://www.revrobotics.com/rev-11-1271/

  public final static int NEO_CURRENT_LIMIT = 80; // amps

  public static final double[][] LookupTable = { // {distance, rpm, angle}
    {5, 0, 108.9651971},
    {4.258293028, 0, 111.8535508},
    {3.679189279, 0, 114.7419044},
    {3.211524819, 0, 117.6302581},
    {2.823481946, 0, 120.5186117},
    {2.494224533, 0, 123.4069653},
    {2.209527097, 0, 126.295319},
    {1.959336833, 0, 129.1836726},
    {1.736339672, 0, 132.0720263},
    {1.535078829, 0, 134.9603799},
    {1.351392365, 0, 137.8487335},
    {1.182042588, 0, 140.7370872},
    {1.024464942, 0, 143.6254408},
    {0.8765935905, 0,	146.5137945},
    {0.7367375292, 0,	149.4021481},
    {0.6034907932, 0,	152.2905018},
    {0.4756661014, 0,	155.1788554},
    {0.3522449123, 0,	158.067209},
    {0.2323391272, 0,	160.9555627},
    {0.1151611524, 0,	163.8439163},
    {0, 0, 166.73227}
  };
}
