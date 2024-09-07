// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs{
        public boolean isLeaderMotorConnected = true;
        public boolean isFollowerMotorConnected = true;

        public double leaderPositionRotations = 0.0;
        public double velocityRps = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runCurrent(double amps) {}

    public default void stop() {}

    public default void runSetpoint(double setpointRotations, double feedforward) {}

    public default void setPID(double p, double i, double d) {}

    public default void setBrakeMode(boolean enabled) {}

    public default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}

}