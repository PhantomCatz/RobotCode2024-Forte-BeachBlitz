package frc.robot.CatzSubsystems.Shooter.ShooterFlywheels;

import static frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.FlywheelConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.CatzConstants;

public class FlywheelsIOSim implements FlywheelsIO {
  private final FlywheelSim leftSim =
      new FlywheelSim(DCMotor.getKrakenX60Foc(1), flywheelConfig.reduction(), 0.00363458292);
  private final FlywheelSim rightSim =
      new FlywheelSim(DCMotor.getKrakenX60Foc(1), flywheelConfig.reduction(), 0.00363458292);

  private final PIDController leftController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final PIDController rightController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRpm = null;
  private Double rightSetpointRpm = null;
  private double leftFeedforward = 0.0;
  private double rightFeedforward = 0.0;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    leftSim.update(CatzConstants.LOOP_TIME);
    rightSim.update(CatzConstants.LOOP_TIME);
    // control to setpoint
    if (leftSetpointRpm != null && rightSetpointRpm != null) {
      runVolts(
          leftController.calculate(leftSim.getAngularVelocityRPM(), leftSetpointRpm)
              + leftFeedforward,
          rightController.calculate(rightSim.getAngularVelocityRPM(), rightSetpointRpm)
              + rightFeedforward);
    }

    inputs.leftPositionRads +=
        Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * CatzConstants.LOOP_TIME);
    inputs.leftVelocityRpm = leftSim.getAngularVelocityRPM();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftSupplyCurrentAmps = leftSim.getCurrentDrawAmps();

    inputs.rightPositionRads +=
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec() * CatzConstants.LOOP_TIME);
    inputs.rightVelocityRpm = rightSim.getAngularVelocityRPM();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    leftSim.setInputVoltage(leftAppliedVolts);
    rightSim.setInputVoltage(rightAppliedVolts);
  }

  @Override
  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    leftSetpointRpm = leftRpm;
    rightSetpointRpm = rightRpm;
    this.leftFeedforward = leftFeedforward;
    this.rightFeedforward = rightFeedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    leftController.setPID(kP, kI, kD);
    rightController.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    runVolts(0.0, 0.0);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    runVolts(input, 0.0);
  }

  @Override
  public void runCharacterizationRight(double input) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    runVolts(0.0, input);
  }
}
