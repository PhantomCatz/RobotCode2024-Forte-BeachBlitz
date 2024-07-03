// 2637
// https://github.com/PhantomCatz/

package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.*;

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
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;

import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

public class ModuleIORealFoc implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final CANSparkMax steerNeo;
  private final DutyCycleEncoder steerAbsoluteMagEnc;
  private final DigitalInput magEncPWMInput;
  private final Rotation2d absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  // Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);
  private final PIDController turnFeedback =
      new PIDController(0.4, 0.1, 0.0, CatzConstants.LOOP_TIME);

  // Status Code Initialization
  private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

  public ModuleIORealFoc(ModuleConfig config) {
    // Init controllers and encoders from config constants
    driveTalon = new TalonFX(config.driveID(), "*");
    steerNeo = new CANSparkMax(config.turnID(), MotorType.kBrushless);
    magEncPWMInput = new DigitalInput(config.absoluteEncoderChannel());
    steerAbsoluteMagEnc = new DutyCycleEncoder(magEncPWMInput);

    absoluteEncoderOffset = Rotation2d.fromRotations(config.absoluteEncoderOffset());

    // Restore Factory Defaults
    driveTalon.getConfigurator().apply(new TalonFXConfiguration());
    steerNeo.restoreFactoryDefaults();

    // Config Motors
    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    steerNeo.restoreFactoryDefaults();
    steerNeo.setSmartCurrentLimit(30);
    steerNeo.setIdleMode(IdleMode.kCoast);
    steerNeo.enableVoltageCompensation(12.0);
    steerNeo.setInverted(false);

    // Conversions affect getPosition()/setPosition() and getVelocity()
    driveTalonConfig.Feedback.SensorToMechanismRatio = moduleGainsAndRatios.driveReduction();

    turnFeedback.enableContinuousInput(-Math.PI, Math.PI);

        //check if drive motor is initialized correctly
        for(int i=0;i<5;i++){
          initializationStatus = driveTalon.getConfigurator().apply(driveTalonConfig);
          if(!initializationStatus.isOK())
              System.out.println("Failed to Configure CAN ID" + config.driveID());
      }

    // 250hz signals
    drivePosition = driveTalon.getPosition();
    // Get signals and set update rate
    // 100hz signals
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent);

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    //---------------------------------------------------------------------------
    // Refresh Drive Kraken status signals
    //---------------------------------------------------------------------------
    //vs
    // Refresh Drive Kraken status signals
    inputs.isDriveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
            .isOK();

    //Drive Input refresh
    inputs.drivePositionUnits = drivePosition.getValueAsDouble();
    inputs.driveVelocityRPS = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    // Refresh Turn Motor Values
    inputs.isTurnMotorConnected = true; //TODO need to find better way of ensuring neos are connected
    inputs.turnAbsolutePosition = 
        Rotation2d.fromRotations(
            steerAbsoluteMagEnc.getAbsolutePosition()-absoluteEncoderOffset.getRotations()
                                ); //TODO getAbsoultePosition() vs get()
    inputs.turnPosition = Rotation2d.fromRotations(steerNeo.getEncoder().getPosition());
    inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(steerNeo.getEncoder().getVelocity());
    inputs.turnBussVolts = steerNeo.getBusVoltage();
    inputs.turnSupplyCurrentAmps = -999.0;

    //steer Enc input collection

    inputs.odometryDrivePositionsMeters = new double[] {drivePosition.getValueAsDouble() * driveConfig.wheelRadius()};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnAbsolutePosition};
  }

  public void runDriveVolts(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  public void runSteerVolts(double volts) {
    steerNeo.setVoltage(volts);
  }

  @Override
  public void runCharacterization(double input) {
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocityRPSIO(double velocityMetersPerSec, double feedForward) {
    driveTalon.setControl(
        velocityTorqueCurrentFOC
            .withVelocity(velocityMetersPerSec)
            .withFeedForward(feedForward)); //In Amps
  }

  @Override
  public void runSteerPositionSetpoint(double currentAngleRad, double targetAngleRad) {
        //calculate steer pwr
        //negative steer power because of coordinate system
    runSteerVolts(-turnFeedback.calculate(currentAngleRad, targetAngleRad));
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveBreakModeIO(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveTalonConfig) {
            driveTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            driveTalon.getConfigurator().apply(driveTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void setSteerBrakeModeIO(boolean enable) {
    if(enable) {
      steerNeo.setIdleMode(IdleMode.kBrake);
  } else {
      steerNeo.setIdleMode(IdleMode.kCoast);
  }
  }

}
