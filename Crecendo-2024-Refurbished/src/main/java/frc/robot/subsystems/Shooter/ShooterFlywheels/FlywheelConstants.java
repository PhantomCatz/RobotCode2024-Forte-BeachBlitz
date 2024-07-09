package frc.robot.subsystems.Shooter.ShooterFlywheels;

import frc.robot.CatzConstants;
import frc.robot.util.LoggedTunableNumber;

public class FlywheelConstants {
  public final static boolean isShooterFlywheelsDisabled = false;

  public static final FlywheelConfig flywheelConfig =
      switch (CatzConstants.getRobotType()) {
        case SN2 -> new FlywheelConfig(4, 0, (1.0 / 2.0), 9000.0);
        case SN1 -> new FlywheelConfig(5, 4, (1.0 / 2.0), 6000.0);
        case SIM -> new FlywheelConfig(0, 0, (1.0 / 2.0), 9000.0);
      };

  public static final Gains gains =
      switch (CatzConstants.getRobotType()) {
        case SN2 -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
        case SN1 -> new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0);
        case SIM -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
      };

  // PID gains
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  public static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  public static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  public static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

  // Adjustable flywheel RPM values
  public static final LoggedTunableNumber shootingLeftRpm =
      new LoggedTunableNumber("Flywheels/ShootingLeftRpm", 5066.0);
  public static final LoggedTunableNumber shootingRightRpm =
      new LoggedTunableNumber("Flywheels/ShootingRightRpm", 7733.0);
  public static final LoggedTunableNumber prepareShootMultiplier =
      new LoggedTunableNumber("Flywheels/PrepareShootMultiplier", 1.0);
  public static final LoggedTunableNumber intakingRpm =
      new LoggedTunableNumber("Flywheels/IntakingRpm", -3000.0);
  public static final LoggedTunableNumber ejectingRpm =
      new LoggedTunableNumber("Flywheels/EjectingRpm", 1000.0);
  public static final LoggedTunableNumber poopingRpm =
      new LoggedTunableNumber("Flywheels/PoopingRpm", 3000.0);
  public static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber(
          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());


  //-----------------------------------------------------------------------------------------------------
  //    Flywheel record
  //-----------------------------------------------------------------------------------------------------
  public record FlywheelConfig(
      int leftID, int rightID, double reduction, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
