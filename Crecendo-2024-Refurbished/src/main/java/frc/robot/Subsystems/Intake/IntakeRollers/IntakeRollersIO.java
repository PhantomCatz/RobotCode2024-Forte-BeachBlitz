package frc.robot.subsystems.Intake.IntakeRollers;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersIOInputs {
        public boolean isRollerMotorConnected = true;

        public double rollerVelocityRpm = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerSupplyCurrentAmps = 0.0;
        public double rollerTorqueCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;


        public boolean isFrontBeambreakBroken;
    }
    
    public default void updateInputs(IntakeRollersIOInputs inputs) {}

    /** Run motors at volts */
    public default void runVolts(double volts) {}

    /** Run motors at duty cycle percentage */
    public default void runDutycycle(double percentOutput) {}

    /** Stops motors */
    default void stop() {}
}
