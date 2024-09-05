package frc.robot.subsystems.SuperSubsystem.ShooterTurret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    
    @AutoLog
    class TurretIOInputs {
        public boolean isTurretMotorConnected;

        public double velocityRps;
        public double positionDegrees;
        public double appliedVolts;
        public double tempCelsius;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runPercentOutput(double percentOutput) {}

    public default void runSetpointDegrees(double currentAngleDegrees, double setpointAngleDegrees) {}

    public default void resetEncoder(double position) {}
}
