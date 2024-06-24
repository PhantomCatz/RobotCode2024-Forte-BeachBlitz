// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public boolean isLeaderMotorConnected = true;
        public boolean isFollowerMotorConnected = true;

        public double positionRotations = 0.0;
        public double velocityRotPerSec = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
    }
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run to setpoint enc in rotations */
    default void runSetpoint(double setpointRotations, double feedforward) {}

    /** Run motors at volts */
    default void runVolts(double volts) {}

    /** Run motors at current */
    default void runCurrent(double amps) {}

    /** Set brake mode enabled */
    default void setBrakeMode(boolean enabled) {}

    /** Set PID values */
    default void setPID(double p, double i, double d) {}

    /** Set Motion Magic values */
    default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

    /** Stops motors */
    default void stop() {}

}