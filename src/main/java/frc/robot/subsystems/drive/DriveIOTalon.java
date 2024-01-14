// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class DriveIOTalon implements DriveIO {
    private final WPI_VictorSPX leftLeader = new WPI_VictorSPX(4);
    private final WPI_VictorSPX leftFollower = new WPI_VictorSPX(5);
    private final WPI_VictorSPX rightLeader = new WPI_VictorSPX(6);
    private final WPI_VictorSPX rightFollower = new WPI_VictorSPX(8);

    private GyroIOReal gyro;

    public DriveIOTalon() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        leftFollower.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Coast);
        rightLeader.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Coast);

        gyro = GyroIOReal.getInstance();
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.gyroYawRad = gyro.getYawAngle();
        inputs.leftPositionRad = 0;
        inputs.rightPositionRad = 0;
        inputs.leftVelocityRadPerSec = leftLeader.getBusVoltage();
        inputs.rightVelocityRadPerSec = rightLeader.getBusVoltage();
        inputs.gyroRollPitchYawRad[0] = gyro.getRollAngle();
        inputs.gyroRollPitchYawRad[1] = gyro.getPitchAngle();
        inputs.gyroRollPitchYawRad[2] = gyro.getYawAngle();
        inputs.timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.set(leftVolts);
        rightLeader.set(rightVolts);
    }

    /* No encoders so no position or velocity control works */
    @Override
    public double getLeftPositionMeters() {
        return 0;
    }

    @Override
    public double getRightPositionMeters() {
        return 0;
    }

    @Override
    public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
    }

    @Override
    public void zero() {
        GyroIOReal.getInstance().zeroAll();
    }

    @Override
    public double getRobotAngle() {
        return GyroIOReal.getInstance().getYawAngle();
    }
}