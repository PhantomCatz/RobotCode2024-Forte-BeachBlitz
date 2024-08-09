// 2637
// https://github.com/PhantomCatz/

package frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.CatzConstants;
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;

import static frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.*;

import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class ModuleIORealFoc implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX steerTalon;
  private final DutyCycleEncoder steerAbsoluteMagEnc;
  private final DigitalInput magEncPWMInput;
  private final Rotation2d absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  private final StatusSignal<Double> steerPosition;
  private final Supplier<Rotation2d> steerAbsolutePosition;
  private final StatusSignal<Double> steerVelocity;
  private final StatusSignal<Double> steerAppliedVolts;
  private final StatusSignal<Double> steerSupplyCurrent;
  private final StatusSignal<Double> steerTorqueCurrent;

  // Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration steerTalonConfig = new TalonFXConfiguration();

  // Control
  private final VoltageOut voltageControl = 
      new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC currentControl = 
      new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut neutralControl = 
      new NeutralOut().withUpdateFreqHz(0);
  private final PIDController steerFeedback =
      new PIDController(0.4, 0.1, 0.0, CatzConstants.LOOP_TIME);

  // Status Code Initialization
  private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

  public ModuleIORealFoc(ModuleConfig config) {
    // Init controllers and encoders from config constants
    driveTalon = new TalonFX(config.driveID());
    steerTalon = new TalonFX(config.steerID());
    magEncPWMInput = new DigitalInput(config.absoluteEncoderChannel());
    steerAbsoluteMagEnc = new DutyCycleEncoder(magEncPWMInput);

    absoluteEncoderOffset = Rotation2d.fromRotations(config.absoluteEncoderOffset());

    // Restore Factory Defaults
    driveTalon.getConfigurator().apply(new TalonFXConfiguration());
    steerTalon.getConfigurator().apply(new TalonFXConfiguration());

    // Config Motors Current Limits assume FOC is included with motors
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    steerTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    steerTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    steerTalonConfig.MotorOutput.Inverted =
        config.steerMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    steerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Conversions affect getPosition()/setPosition() and getVelocity()
    driveTalonConfig.Feedback.SensorToMechanismRatio = moduleGainsAndRatios.driveReduction();

    steerFeedback.enableContinuousInput(-Math.PI, Math.PI);

    //check if drive motor is initialized correctly
    for(int i=0;i<5;i++){
      initializationStatus = driveTalon.getConfigurator().apply(driveTalonConfig);
      if(!initializationStatus.isOK())
          System.out.println("Failed to Configure CAN ID" + config.driveID());
    }

    // Assign 100hz Signals
    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    steerPosition = steerTalon.getPosition();
    steerAbsolutePosition =
        () -> Rotation2d.fromRotations(steerAbsoluteMagEnc.getAbsolutePosition())
                .minus(absoluteEncoderOffset);
    steerVelocity = steerTalon.getVelocity();
    steerAppliedVolts = steerTalon.getMotorVoltage();
    steerSupplyCurrent = steerTalon.getSupplyCurrent();
    steerTorqueCurrent = steerTalon.getTorqueCurrent();


    // Set Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        steerVelocity,
        steerAppliedVolts,
        steerSupplyCurrent,
        steerTorqueCurrent
    );

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {

    // Refresh Drive Kraken status signals
    inputs.isDriveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
            .isOK();

    //Drive Input variable refresh
    inputs.drivePositionUnits = drivePosition.getValueAsDouble();
    inputs.driveVelocityRPS = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    // Refresh steer Motor Values
    inputs.isSteerMotorConnected = 
        BaseStatusSignal.refreshAll(
                steerPosition, 
                steerVelocity, 
                steerAppliedVolts, 
                steerSupplyCurrent, 
                steerTorqueCurrent)
            .isOK();
    inputs.steerAbsolutePosition = steerAbsolutePosition.get();
    inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerSupplyCurrentAmps = steerSupplyCurrent.getValueAsDouble();
    inputs.steerTorqueCurrentAmps = steerTorqueCurrent.getValueAsDouble();

    inputs.odometryDrivePositionsMeters = new double[] {drivePosition.getValueAsDouble() * driveConfig.wheelRadius()};
    inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerAbsolutePosition};
  }

  public void runDriveVolts(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  public void runSteerVolts(double volts) {
    steerTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCharacterization(double input) {
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityMetersPerSec, double feedForward) { //TODO
    driveTalon.setControl(velocityTorqueCurrentFOC.withVelocity(velocityMetersPerSec)
                                                  .withFeedForward(feedForward) //In Amps
    );
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRad, double targetAngleRad) {
      //calculate steer pwr
      //negative steer power because of coordinate system
    double volts = -steerFeedback.calculate(currentAngleRad, targetAngleRad); 
    runSteerVolts(volts);
    Logger.recordOutput("Drive/steer Output Volts", volts);
    Logger.recordOutput("Drive/steer current Angle", currentAngleRad);
    Logger.recordOutput("Drive/steer Target Angle", targetAngleRad);

  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveBrakeModeIO(boolean enable) {
    driveTalonConfig.MotorOutput.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(driveTalonConfig);
  }

  @Override
  public void setSteerBrakeModeIO(boolean enable) {
    steerTalonConfig.MotorOutput.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    steerTalon.getConfigurator().apply(steerTalonConfig);
  }

}
