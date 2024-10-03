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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.CatzConstants;
import frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.TurretConstants;

public class ShooterPivotIOReal implements ShooterPivotIO {

    // Hardware
    private final CANSparkMax elevationNeoMtr;

    // Control
    private final PIDController ShooterPivotFeedback = new PIDController(gains.kP(), gains.kI(), gains.kD(), CatzConstants.LOOP_TIME);

    public ShooterPivotIOReal() {
        elevationNeoMtr = new CANSparkMax(0, MotorType.kBrushless);

        elevationNeoMtr.restoreFactoryDefaults();
        elevationNeoMtr.setSmartCurrentLimit(30);
        elevationNeoMtr.setIdleMode(IdleMode.kBrake);
        elevationNeoMtr.enableVoltageCompensation(12.0);
        elevationNeoMtr.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.positionDegrees = Units.rotationsToDegrees(elevationNeoMtr.getEncoder().getPosition());
        inputs.velocityRpm = elevationNeoMtr.getEncoder().getVelocity();
        inputs.appliedVolts = elevationNeoMtr.getBusVoltage();
        inputs.tempCelcius = elevationNeoMtr.getMotorTemperature();
    }

    //-----------------------------------------------------------------------------------------
    //
    //    Pivot Power Output
    //
    //-----------------------------------------------------------------------------------------
    @Override
    public void runSetpointDegrees(double currentAngleDegrees,double setpointAngleDegrees) {
        double percentOutput = ShooterPivotFeedback.calculate(currentAngleDegrees, setpointAngleDegrees);
        elevationNeoMtr.set(percentOutput);
    }
  
    @Override
    public void runVolts(double volts) {
      elevationNeoMtr.setVoltage(volts);
    }

  
    @Override
    public void stop() {
      elevationNeoMtr.set(0);
    }
}
