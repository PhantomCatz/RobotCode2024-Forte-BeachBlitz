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
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;

import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class ModuleIORealFoc implements ModuleIO 
{
  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX steerTalon;

  private final DutyCycleEncoder steerAbsoluteMagEnc;
  private final DigitalInput     magEncPWMInput;
  private final Rotation2d       absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  private final Supplier<Rotation2d> steerAbsolutePosition;
  private final StatusSignal<Double> steerPosition;
  private final StatusSignal<Double> steerVelocity;
  private final StatusSignal<Double> steerAppliedVolts;
  private final StatusSignal<Double> steerSupplyCurrent;
  private final StatusSignal<Double> steerTorqueCurrent;

  // Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();   //This should go with TalonFX Decl above
  private final TalonFXConfiguration steerTalonConfig = new TalonFXConfiguration();

  // Control
  private final VoltageOut               voltageControl           = new VoltageOut              (0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC         currentControl           = new TorqueCurrentFOC        (0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionControl          = new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut               neutralControl           = new NeutralOut              ( ).withUpdateFreqHz(0);

  private final PIDController            steerFeedback            = new PIDController(0.4, 0.1, 0.0, CatzConstants.LOOP_TIME);  //TBD how do these values relate to DriveConstants values

  // Status Code Initialization
  private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;




  public ModuleIORealFoc(ModuleConfig config) 
  {

    //----------------------------------------------------------------------------------------------
    // 
    //  Init & Config Drivetrain motor controllers   Power Setting methods
    // 
    //----------------------------------------------------------------------------------------------
    steerSparkMax       = new TalonFX(config.steerID());

    // Restore Factory Defaults
    steerTalon.getConfigurator().apply(new TalonFXConfiguration());

    //----------------------------------------------------------------------------------------------
    //  Drive Motor Config (FOC) - Current limits, Brake Mode
    //----------------------------------------------------------------------------------------------
    driveTalon          = new TalonFX(config.driveID());    //TBD - Don't but MC type in name s/b driveMC or driveMtrCntlr
    driveTalon.getConfigurator().apply(new TalonFXConfiguration());


    driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent =  80.0;      //TBD Why are these not in Drive Constants?
    driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;

    driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    
    driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


    // Conversions affect getPosition()/setPosition() and getVelocity()
    driveTalonConfig.Feedback.SensorToMechanismRatio = moduleGainsAndRatios.driveReduction();

    //check if drive motor is initialized correctly
    for(int i=0;i<5;i++)                                        //TBD - Why 5?
    {
      initializationStatus = driveTalon.getConfigurator().apply(driveTalonConfig);

      if(!initializationStatus.isOK())
          System.out.println("Failed to Configure CAN ID" + config.driveID());
    }

    //----------------------------------------------------------------------------------------------
    //  Steer Motor Config - Current limits, Inversion, Brake Mode
    //----------------------------------------------------------------------------------------------
    steerTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent =  40.0;      //TBD See above, also how did we come up with 40?
    steerTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;

    if (config.steerMotorInverted() == true) {
      steerTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      steerTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    steerTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //----------------------------------------------------------------------------------------------
    //  Init Drivetrain steering encoders 
    //----------------------------------------------------------------------------------------------
    magEncPWMInput      = new DigitalInput(config.absoluteEncoderChannel());

    steerAbsoluteMagEnc = new DutyCycleEncoder(magEncPWMInput);

    absoluteEncoderOffset = Rotation2d.fromRotations(config.absoluteEncoderOffset());

    steerFeedback.enableContinuousInput(-Math.PI, Math.PI);


    //----------------------------------------------------------------------------------------------
    //  TBD
    //----------------------------------------------------------------------------------------------
    // Assign 100hz Signals                                 //TBD what does comment mean?
    drivePosition      = driveTalon.getPosition();          //TBD this is not listed in setUpdateFrequencyForAll
 
    driveVelocity      = driveTalon.getVelocity();
    driveAppliedVolts  = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();

    steerPosition         = steerTalon.getPosition();       //TBD this is not listed in setUpdateFrequencyForAll
    steerAbsolutePosition = () -> Rotation2d.fromRotations(steerAbsoluteMagEnc.getAbsolutePosition())
                                                                              .minus(absoluteEncoderOffset);            //TBD this is not listed in setUpdateFrequencyForAll
    steerVelocity         = steerTalon.getVelocity();
    steerAppliedVolts     = steerTalon.getMotorVoltage();
    steerSupplyCurrent    = steerTalon.getSupplyCurrent();
    steerTorqueCurrent    = steerTalon.getTorqueCurrent();


    // Set Update Frequency
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,                  //10msec poll rate
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,

        steerVelocity,
        steerAppliedVolts,
        steerSupplyCurrent,
        steerTorqueCurrent
    );

    // Optimize bus utilization                     //TBD not helpful
    driveTalon.optimizeBusUtilization(0, 1.0);      //TBD Lets turn this off for now,  what do these parameters mean??
  
}   //End of ModuleIORealFoc



  //------------------------------------------------------------------------------------------------
  // 
  //  updateInputs()
  // 
  //------------------------------------------------------------------------------------------------
  @Override
  public void updateInputs(ModuleIOInputs inputs) 
  {

    // Refresh Drive Kraken status signals
    inputs.isDriveMotorConnected = BaseStatusSignal.refreshAll(drivePosition,
                                                               driveVelocity,
                                                               driveAppliedVolts,
                                                               driveSupplyCurrent,
                                                               driveTorqueCurrent ).isOK();

    //Drive Input variable refresh
    inputs.drivePositionUnits     = drivePosition     .getValueAsDouble();
    inputs.driveVelocityRPS       = driveVelocity     .getValueAsDouble();
    inputs.driveAppliedVolts      = driveAppliedVolts .getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    // Refresh steer Motor Values
    inputs.isSteerMotorConnected = BaseStatusSignal.refreshAll(steerPosition, 
                                                               steerVelocity, 
                                                               steerAppliedVolts, 
                                                               steerSupplyCurrent, 
                                                               steerTorqueCurrent ).isOK();

    inputs.steerAbsolutePosition   = steerAbsolutePosition.get();
    inputs.steerPosition           = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
    inputs.steerVelocityRadsPerSec = Units.rotationsToRadians(steerVelocity.getValueAsDouble());

    inputs.steerAppliedVolts       = steerAppliedVolts .getValueAsDouble();
    inputs.steerSupplyCurrentAmps  = steerSupplyCurrent.getValueAsDouble();
    inputs.steerTorqueCurrentAmps  = steerTorqueCurrent.getValueAsDouble();

    inputs.odometryDrivePositionsMeters = new double[]     {drivePosition.getValueAsDouble() * driveConfig.wheelRadius()};  //TBD is this really an array
    inputs.odometrySteerPositions       = new Rotation2d[] {inputs.steerAbsolutePosition};

  }   //End of updateInputs()



  //------------------------------------------------------------------------------------------------
  // 
  //  TBD Methods
  // 
  //------------------------------------------------------------------------------------------------
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
  public void runDriveVelocityRPSIO(double velocityMetersPerSec, double feedForward) 
  { //TODO
    driveTalon.setControl(velocityTorqueCurrentFOC.withVelocity   (velocityMetersPerSec)
                                                  .withFeedForward(feedForward         ) );//In Amps   TBD - how is this format
  }


  @Override
  public void runSteerPositionSetpoint(double currentAngleRad, double targetAngleRad) 
  {
        //calculate steer pwr
        //negative steer power because of coordinate system
    double volts = -steerFeedback.calculate(currentAngleRad, targetAngleRad); 
    runSteerVolts(volts);

    Logger.recordOutput("Drive/steer Output Volts",  volts);
    Logger.recordOutput("Drive/steer current Angle", currentAngleRad);
    Logger.recordOutput("Drive/steer Target Angle",  targetAngleRad);
  }


  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);     //TBD - Timeout
  }


  @Override
  public void setSteerPID(double kP, double kI, double kD) {
    steerFeedback.setPID(kP, kI, kD);
  }


  @Override
  public void setDriveNeutralMode(enum mode) 
  {
    if(mode == BrakeMode) {
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
    driveTalon.getConfigurator().apply(driveTalonConfig);
  }


  @Override
  public void setDriveBrakeModeIO(boolean enable)           //TBD
  {
    driveTalonConfig.MotorOutput.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(driveTalonConfig);
  }


  @Override
  public void setSteerBrakeModeIO(boolean enable) 
  {
    steerTalonConfig.MotorOutput.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    steerTalon.getConfigurator().apply(steerTalonConfig);
  }

}