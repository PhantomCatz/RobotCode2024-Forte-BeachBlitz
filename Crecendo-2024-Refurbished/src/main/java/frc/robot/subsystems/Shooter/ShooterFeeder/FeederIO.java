package frc.robot.subsystems.Shooter.ShooterFeeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    
    @AutoLog
    public static class FeederIOInputs{
        public boolean isAdjustBeamBreakBroken;
        public boolean isLoadBeamBreakBroken;
        public double  noteMovementUpInches;
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void loadBackward() {}

    public default void loadFoward() {}

    public default void fineAdjustBck() {}

    public default void fineAdjustFwd() {}

    public default void feedDisabled() {}

    public default void feedShooter() {}
    
    public default void resetLoadEnc() {}



}
