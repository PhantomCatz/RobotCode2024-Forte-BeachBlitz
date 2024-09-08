package frc.robot.CatzSubsystems.Shooter.ShooterFlywheels;

import static frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.FlywheelConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelsIOReal implements FlywheelsIO {

  // Hardware
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  // Status Signals
  private final StatusSignal<Double> leftPosition;
  private final StatusSignal<Double> leftVelocity;
  private final StatusSignal<Double> leftAppliedVolts;
  private final StatusSignal<Double> leftSupplyCurrent;
  private final StatusSignal<Double> leftTorqueCurrent;
  private final StatusSignal<Double> leftTempCelsius;
  private final StatusSignal<Double> rightPosition;
  private final StatusSignal<Double> rightVelocity;
  private final StatusSignal<Double> rightAppliedVolts;
  private final StatusSignal<Double> rightSupplyCurrent;
  private final StatusSignal<Double> rightTorqueCurrent;
  private final StatusSignal<Double> rightTempCelsius;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public FlywheelsIOReal() {
    leftTalon = new TalonFX(flywheelConfig.leftID());
    rightTalon = new TalonFX(flywheelConfig.rightID());

    // General Talon Config
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = flywheelConfig.reduction();

    // PIDF Talon config
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();

    // Apply configs
    leftTalon.getConfigurator().apply(config, 1.0);
    rightTalon.getConfigurator().apply(config, 1.0);

    // Set inverts
    leftTalon.setInverted(true);
    rightTalon.setInverted(false);

    // Assign signals
    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTorqueCurrent = leftTalon.getTorqueCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();

    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTorqueCurrent = rightTalon.getTorqueCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftSupplyCurrent,
        leftTorqueCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightSupplyCurrent,
        rightTorqueCurrent,
        rightTempCelsius
      );

    leftTalon.optimizeBusUtilization(0, 1.0);
    rightTalon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.isLeftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition,
                leftVelocity,
                leftAppliedVolts,
                leftSupplyCurrent,
                leftTorqueCurrent,
                leftTempCelsius)
            .isOK();
    inputs.isRightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTorqueCurrent,
                rightTempCelsius)
            .isOK();

    inputs.leftPositionRads = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

    inputs.rightPositionRads = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Flywheel Power Output
  //
  //-----------------------------------------------------------------------------------------
  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftTalon.setControl(voltageControl.withOutput(leftVolts));
    rightTalon.setControl(voltageControl.withOutput(rightVolts));
  }

  @Override
  public void stop() {
    leftTalon.setControl(neutralControl);
    rightTalon.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    leftTalon.setControl(
        velocityControl
            .withVelocity(leftRpm / 60.0)
            .withFeedForward(leftFeedforward)
    );
    rightTalon.setControl(
          velocityControl
              .withVelocity(rightRpm / 60.0)
              .withFeedForward(rightFeedforward)
    );
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Flywheel Misc
  //
  //-----------------------------------------------------------------------------------------
  @Override
  public void setPID(double kP, double kI, double kD) {
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    leftTalon.getConfigurator().apply(config);
    rightTalon.getConfigurator().apply(config);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftTalon.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void runCharacterizationRight(double input) {
    rightTalon.setControl(voltageControl.withOutput(input));
  }
}
