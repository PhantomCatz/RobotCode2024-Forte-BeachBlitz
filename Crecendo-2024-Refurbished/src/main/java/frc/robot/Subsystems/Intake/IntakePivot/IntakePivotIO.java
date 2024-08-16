// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/** Add your docs here. */
public interface IntakePivotIO{
    @AutoLog
    public static class IntakePivotIOInputs {
        public boolean isPivotMotorConnected = true;

        public double positionRads;
        public double velocityRps;
        public double closedLoopPivotMtr;
        public double appliedVolts;
        public double supplyCurrentAmps;
        public double torqueCurrentAmps;
        public double tempCelsius = 0.0;
    }
    
    public default void updateInputs(IntakePivotIOInputs inputs) {}

    /** Run motors at volts */
    public default void runVolts(double volts) {}

    /** Run to setpoint enc in degrees */
    public default void runSetpoint(double setpointDegrees, double feedforward) {}

    /** Run motors at current */
    default void runCurrent(double amps) {}

    /** Stops motors */
    default void stop() {}

    /** Set Neutral Mode */
    public default void setNeutralMode(NeutralMode type) {}

    /** Set PID values */
    default void setPID(double p, double i, double d) {}

    /** Config FF values */
    public default void setFF(double kS, double kV, double kA) {}

    /** Set Motion Magic values */
    default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}


}