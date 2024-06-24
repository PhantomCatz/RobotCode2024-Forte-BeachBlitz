package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), DriveConstants.moduleGainsAndRatios.driveReduction(), 0.025);
  private final DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getKrakenX60Foc(1), DriveConstants.moduleGainsAndRatios.turnReduction(), 0.004);

  private final PIDController driveFeedback =
      new PIDController(0.1, 0.0, 0.0, CatzConstants.LOOP_TIME);
  private final PIDController turnFeedback =
      new PIDController(10.0, 0.0, 0.0, CatzConstants.LOOP_TIME);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final Rotation2d turnAbsoluteInitPosition;

  private boolean driveCoast = false;
  private SlewRateLimiter driveVoltsLimiter = new SlewRateLimiter(2.5);

  public ModuleIOSim(ModuleConfig config) {
    turnAbsoluteInitPosition = Rotation2d.fromRadians(Units.rotationsToRadians(config.absoluteEncoderOffset()));
    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    if (driveCoast && DriverStation.isDisabled()) {
      runDriveVolts(driveVoltsLimiter.calculate(driveAppliedVolts));
    } else {
      driveVoltsLimiter.reset(driveAppliedVolts);
    }

    driveSim.update(CatzConstants.LOOP_TIME);
    turnSim.update(CatzConstants.LOOP_TIME);

    inputs.driveVelocityRPS =   driveSim.getAngularVelocityRPM()/60; //Convert to RPS
    inputs.drivePositionUnits = driveSim.getAngularPositionRad()/(2*Math.PI);
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnBussVolts = turnAppliedVolts;
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryDrivePositionsMeters =
        new double[] {driveSim.getAngularPositionRad() * DriveConstants.driveConfig.wheelRadius()};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRadians(turnSim.getAngularPositionRad())};

  }

  private void runDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  private void runTurnVolts(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void runCharacterization(double input) {
    runDriveVolts(input);
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityRPS, double feedForward) {
    double velocityRadsPerSec = Units.rotationsToRadians(velocityRPS);
    runDriveVolts(
        driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(), velocityRadsPerSec)
            + feedForward);
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRad, double angleRads) {
    runTurnVolts(turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveBreakModeIO(boolean enable) {
    driveCoast = !enable;
  }
}
