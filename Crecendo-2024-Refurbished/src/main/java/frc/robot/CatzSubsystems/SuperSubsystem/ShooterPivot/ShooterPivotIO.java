package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ShooterPivotIO {
    
    @AutoLog
    class ShooterPivotIOInputs {
        public boolean isElevationMotorConnected = true;

        public double positionDegrees = 0.0;
        public double velocityRpm = 0.0;
        public double appliedVolts = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelcius = 0.0;
    }

    public default void updateInputs(ShooterPivotIOInputs inputs) {}
    
    public default void runVolts(double volts) {}

    public default void stop() {}

    public default void runSetpointDegrees(double currentAngleDegrees,double setpointAngleDegrees) {}
}
