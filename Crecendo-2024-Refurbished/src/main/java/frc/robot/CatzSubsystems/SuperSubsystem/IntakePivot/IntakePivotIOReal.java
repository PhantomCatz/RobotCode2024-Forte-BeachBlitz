// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import static frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.IntakePivotConstants.PIVOT_MTR_ID;
import static frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.IntakePivotConstants.gains;
import static frc.robot.CatzSubsystems.SuperSubsystem.IntakePivot.IntakePivotConstants.motionMagicParameters;

/** Add your docs here. */
public class IntakePivotIOReal implements IntakePivotIO {

  // Hardware
  private final TalonFX pivotTalon;

  // Status Signals
  private final StatusSignal<Double> position;
  private final StatusSignal<Double> velocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> supplyCurrent;
  private final StatusSignal<Double> torqueCurrent;
  private final StatusSignal<Double> tempCelsius;


  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
  private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();


  public IntakePivotIOReal() {
    pivotTalon = new TalonFX(PIVOT_MTR_ID);

    // General config
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //config.Feedback.SensorToMechanismRatio = FINAL_REDUCTION;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    // Controller config;
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kS = gains.kS();
    config.Slot0.kV = gains.kV();
    config.Slot0.kA = gains.kA();
    

    config.MotionMagic.MotionMagicCruiseVelocity = motionMagicParameters.mmCruiseVelocity(); // Target cruise velocity of 80 rps
    config.MotionMagic.MotionMagicAcceleration   = motionMagicParameters.mmAcceleration(); // Target acceleration of 400 rps/s (0.5 seconds)
    config.MotionMagic.MotionMagicJerk           = motionMagicParameters.mmJerk(); // Target jerk of 1600 rps/s/s (0.1 seconds)

    pivotTalon.setPosition(IntakePivotConstants.INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV);
    System.out.println(pivotTalon.getPosition().getValue());

    // Apply configs
    pivotTalon.getConfigurator().apply(config, 1.0);

    // Set signals
    position = pivotTalon.getPosition();
    velocity = pivotTalon.getVelocity();
    appliedVolts = pivotTalon.getMotorVoltage();
    supplyCurrent = pivotTalon.getSupplyCurrent();
    torqueCurrent = pivotTalon.getTorqueCurrent();
    tempCelsius = pivotTalon.getDeviceTemp();


    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        torqueCurrent,
        tempCelsius
    );
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    inputs.isPivotMotorConnected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                appliedVolts,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();

    inputs.positionRads = (Units.rotationsToRadians(position.getValueAsDouble()));
    inputs.velocityRps = velocity.getValueAsDouble() * 60.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runSetpoint(double setpointDegrees) {
    pivotTalon.setControl(
        positionControl
            .withPosition(Units.degreesToRotations(setpointDegrees))
    ); 
  }

  @Override
  public void runVolts(double volts) {
    pivotTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCurrent(double amps) {
    pivotTalon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void stop() {
    pivotTalon.setControl(new NeutralOut());
  }

  //-----------------------------------------------------------------------------------------
  //
  //    Elevator Misc
  //
  //-----------------------------------------------------------------------------------------
  @Override
  public void setNeutralMode(NeutralMode type) {
    pivotTalon.setNeutralMode(type == NeutralMode.Brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);

  }

  @Override
  public void setPID(double p, double i, double d) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    pivotTalon.getConfigurator().apply(config, 0.01);
  }

  @Override
  public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {
    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration   = acceleration;
    config.MotionMagic.MotionMagicJerk           = jerk;
    pivotTalon.getConfigurator().apply(config, 0.01);
  }
    
}
