package frc.robot.subsystems.Shooter.ShooterFeeder;

public class ShooterFeederConstants {
    public static final boolean isShooterDisabledFeeder = true;

    // Feeder Ratio
    public static final double FEEDER_GEAR_RATIO = 3.0;
    public static final double FEEDER_COMPLIANT_WHEEL_DIAMETER_INCH = 2.25;

    // Feeder Roller Speeds
    public static final double LOAD_MOTOR_SHOOTING_SPEED   = 1.0;
    public static final double LOAD_MOTOR_LOADING_SPEED    = 0.6;//0.3;//0.6; //was 0.4 LISA
    public static final double LOAD_MOTOR_BACKWARD_SPEED   = 0.8;
    public static final double LOAD_MOTOR_ADJUST_SPEED     = 0.04;
    public static final double LOAD_MOTOR_TRANSFER_SPEED   = 0.4;
}
