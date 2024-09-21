package frc.robot.CatzSubsystems.SuperSubsystem.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;

import static frc.robot.CatzSubsystems.SuperSubsystem.Elevator.ElevatorConstants.*;

import java.util.List;

public class ElevatorIOReal implements ElevatorIO {

  // Hardware
  private final TalonFX leaderTalon;

  // Status Signals
  private final StatusSignal<Double> internalPositionRotations;
  private final StatusSignal<Double> velocityRps;
  private final StatusSignal<Double> appliedVoltage;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> torqueCurrent;
  private final StatusSignal<Double> tempCelsius;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ElevatorIOReal() {
    leaderTalon = new TalonFX(leaderID);

    // General Talon Config
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.Inverted =
        IS_LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // PIDF Talon config
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kP = gains.kA();
    config.Slot0.kI = gains.kV();
    config.Slot0.kD = gains.kG();

    config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity(); // Target cruise velocity of 80 rps
    config.MotionMagic.MotionMagicAcceleration   = motionMagicParameters.mmAcceleration(); // Target acceleration of 400 rps/s (0.5 seconds)
    config.MotionMagic.MotionMagicJerk           = motionMagicParameters.mmJerk(); // Target jerk of 1600 rps/s/s (0.1 seconds)
    
    // Apply Talon Configs
    leaderTalon.getConfigurator().apply(config, 1.0);

    // Assign leader signals
    internalPositionRotations = leaderTalon.getPosition();
    velocityRps = leaderTalon.getVelocity();

    // Assign leader and Follower signals
    appliedVoltage = leaderTalon.getMotorVoltage();
    supplyCurrent = leaderTalon.getSupplyCurrent();
    torqueCurrent = leaderTalon.getTorqueCurrent();
    tempCelsius = leaderTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    leaderTalon.optimizeBusUtilization(0, 1.0);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.isLeaderMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();


    inputs.leaderPositionRotations = internalPositionRotations.getValueAsDouble();
    inputs.velocityRps        = velocityRps.getValue();
    inputs.appliedVolts      = appliedVoltage.getValue();
    inputs.supplyCurrentAmps = supplyCurrent.getValue();
    inputs.torqueCurrentAmps = torqueCurrent.getValue();
    inputs.tempCelcius       = tempCelsius.getValue();
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Elevator Power Output
  //
  //-----------------------------------------------------------------------------------------
  @Override
  public void runSetpoint(double setpointRotations, double feedforward) {
    System.out.println(setpointRotations);
    leaderTalon.setControl(
        positionControl
            .withPosition(setpointRotations)
            .withFeedForward(feedforward)
    );
  }

  @Override
  public void runVolts(double volts) {
    leaderTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    leaderTalon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new NeutralOut());
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Elevator Misc
  //
  //-----------------------------------------------------------------------------------------
  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);

  }

  @Override
  public void setPID(double p, double i, double d) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    leaderTalon.getConfigurator().apply(config, 0.01);
  }

  @Override
  public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {
    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration   = acceleration;
    config.MotionMagic.MotionMagicJerk           = jerk;
    leaderTalon.getConfigurator().apply(config, 0.01);
  }
}
