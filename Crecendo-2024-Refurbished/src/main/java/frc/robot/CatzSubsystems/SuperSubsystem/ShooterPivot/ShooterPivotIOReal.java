package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ShooterPivotIOReal implements ShooterPivotIO {

    // Hardware
    private final TalonFX elevationTalon;

    // Status Signals
    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private final StatusSignal<Double> appliedVolts;
    private final StatusSignal<Double> supplyCurrent;
    private final StatusSignal<Double> torqueCurrent;
    private final StatusSignal<Double> tempCelsius;

    // Control
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public ShooterPivotIOReal() {
        elevationTalon = new TalonFX(0);

        // General Talon Config
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = 0.0;

        // PIDF Talon config
        config.Slot0.kP = gains.kP();
        config.Slot0.kI = gains.kI();
        config.Slot0.kD = gains.kD();
        config.Slot0.kS = gains.kS();
        config.Slot0.kV = gains.kV();
        config.Slot0.kA = gains.kA();

        // Assign signals
        position = elevationTalon.getPosition();
        velocity = elevationTalon.getVelocity();
        appliedVolts = elevationTalon.getMotorVoltage();
        supplyCurrent = elevationTalon.getSupplyCurrent();
        torqueCurrent = elevationTalon.getTorqueCurrent();
        tempCelsius = elevationTalon.getDeviceTemp();

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

    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.isElevationMotorConnected =
            BaseStatusSignal.refreshAll(
                    position,
                    velocity,
                    appliedVolts,
                    supplyCurrent,
                    torqueCurrent,
                    tempCelsius)
                .isOK();
        inputs.positionDegrees = Units.rotationsToDegrees(position.getValue());
        inputs.velocityRpm = velocity.getValue() * 60.0;
        inputs.appliedVolts = appliedVolts.getValue();
        inputs.supplyCurrentAmps = supplyCurrent.getValue();
        inputs.torqueCurrentAmps = torqueCurrent.getValue();
        inputs.tempCelcius = tempCelsius.getValue();
    }

    //-----------------------------------------------------------------------------------------
    //
    //    Pivot Power Output
    //
    //-----------------------------------------------------------------------------------------
    @Override
    public void runMotionMagicSetpoint(double setpoint) {
      elevationTalon.setControl(
          positionControl
              .withPosition(Units.degreesToRotations(setpoint))
      );
    }
  
    @Override
    public void runVolts(double volts) {
      elevationTalon.setControl(voltageControl.withOutput(volts));
    }

  
    @Override
    public void stop() {
      elevationTalon.setControl(new NeutralOut());
    }
  
    //-----------------------------------------------------------------------------------------
    //
    //    Pivot Misc
    //
    //-----------------------------------------------------------------------------------------
    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        elevationTalon.getConfigurator().apply(config);
    }

    @Override
    public void setMotionMagicParameters(double cruiseVelocity, double acceleration, double jerk) {
      config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
      config.MotionMagic.MotionMagicAcceleration   = acceleration;
      config.MotionMagic.MotionMagicJerk           = jerk;
      elevationTalon.getConfigurator().apply(config, 0.01);
    }

    @Override
    public void runCharacterization(double input) {
        elevationTalon.setControl(voltageControl.withOutput(input));
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        elevationTalon.setNeutralMode(mode);
    }
}
