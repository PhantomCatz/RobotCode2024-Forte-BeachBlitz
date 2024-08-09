package frc.robot.Subsystems.Shooter.ShooterTurret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    
    @AutoLog
    public static class TurretIOInputs {
        public double velocityRps;
        public double positionDegrees;
        public double appliedVolts;
        public double tempCelsius;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runPercentOuput(double percentOutput) {}

    public default void runSetpointDegrees(double currentAngleDegrees, double setpointAngleDegrees) {}

    public default void resetEncoder(double position) {}
}
