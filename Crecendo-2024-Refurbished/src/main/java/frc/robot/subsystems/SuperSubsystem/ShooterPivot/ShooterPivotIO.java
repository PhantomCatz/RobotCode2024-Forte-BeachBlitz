package frc.robot.subsystems.SuperSubsystem.ShooterPivot;

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

    public default void runMotionMagicSetpoint(double position) {}

    /** Config PID values for both motors */
    public default void setPID(double kP, double kI, double kD) {}

    /** Config FF values for both motors */
    public default void setFF(double kS, double kV, double kA) {}

    /** Run left flywheels at voltage */
    public default void runCharacterization(double input) {}

    public default void setNeutralMode(NeutralModeValue mode) {}

    public default void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {}


}
