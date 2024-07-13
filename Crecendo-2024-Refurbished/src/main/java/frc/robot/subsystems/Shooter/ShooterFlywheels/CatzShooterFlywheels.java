// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterFlywheels;

import static frc.robot.subsystems.Shooter.ShooterFlywheels.FlywheelConstants.*;

import frc.robot.subsystems.Shooter.ShooterFlywheels.FlywheelConstants;
import frc.robot.utilities.Alert;
import frc.robot.utilities.LinearProfile;
import frc.robot.utilities.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class CatzShooterFlywheels extends SubsystemBase {

  // Hardware IO declaration
  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  // Motion Control Declaration
  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());

  // MISC variables
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;
  @Setter private BooleanSupplier prepareShootSupplier = () -> false;

  @AutoLogOutput(key = "Flywheels/Goal")
  private TargetSpeed speed = TargetSpeed.IDLE;

  // Disconnected alerts
  private final Alert leftDisconnected =
      new Alert("Left flywheel disconnected!", Alert.AlertType.WARNING);
  private final Alert rightDisconnected =
      new Alert("Right flywheel disconnected!", Alert.AlertType.WARNING);
  

  // Flywheel State machine
  @RequiredArgsConstructor
  public enum TargetSpeed {
    IDLE(() -> 0.0, () -> 0.0),
    SHOOT(shootingLeftRpm, shootingRightRpm),
    INTAKE(intakingRpm, intakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    POOP(poopingRpm, poopingRpm),
    HOARD(
        () -> 0,//RobotState.getInstance().getSuperPoopAimingParameters().flywheelSpeeds().leftSpeed(),
        () -> 0),//TODO add auto aim paremeters later
           // RobotState.getInstance().getSuperPoopAimingParameters().flywheelSpeeds().rightSpeed()),
    CHARACTERIZING(() -> 0.0, () -> 0.0);

    private final DoubleSupplier leftGoal;
    private final DoubleSupplier rightGoal;

    private double getLeftTargetSpeed() {
      return leftGoal.getAsDouble();
    }

    private double getRightTargetSpeed() {
      return rightGoal.getAsDouble();
    }
  }

  public enum IdleMode {
    TELEOP,
    AUTO
  }

  public CatzShooterFlywheels() {
     if(FlywheelConstants.isShooterFlywheelsDisabled) {
      io = new FlywheelsIONull();
      System.out.println("Feeder Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new FlywheelsIOReal();
          System.out.println("Flywheels Configured for Real");
        break;
        case REPLAY:
          io = new FlywheelsIOReal() {};
          System.out.println("Flywheels Configured for Replayed simulation");
        break;
        case SIM:
          io = new FlywheelsIOSim();
          System.out.println("Flywheels Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Flywheels Unconfigured");
        break;
      }
    }

    leftProfile = new LinearProfile(maxAcceleration.get(), CatzConstants.LOOP_TIME);
    rightProfile = new LinearProfile(maxAcceleration.get(), CatzConstants.LOOP_TIME);

    setDefaultCommand(runOnce(() -> setSpeed(TargetSpeed.IDLE)).withName("Flywheels Idle"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Set alerts
    leftDisconnected.set(!inputs.leftMotorConnected);
    rightDisconnected.set(!inputs.rightMotorConnected);

    // Check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> ff = new SimpleMotorFeedforward(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setSpeed(TargetSpeed.IDLE);
    }

    // Get goal
    double leftGoal = speed.getLeftTargetSpeed();
    double rightGoal = speed.getRightTargetSpeed();
    boolean idlePrepareShoot = speed == TargetSpeed.IDLE && prepareShootSupplier.getAsBoolean();
    if (idlePrepareShoot) {
      leftGoal = TargetSpeed.SHOOT.getLeftTargetSpeed() * prepareShootMultiplier.get();
      rightGoal = TargetSpeed.SHOOT.getRightTargetSpeed() * prepareShootMultiplier.get();
    }

    // Run to setpoint
    if (closedLoop || idlePrepareShoot) {
      // Update goals
      double leftSetpoint =  leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      io.runVelocity(
          leftSetpoint, rightSetpoint, ff.calculate(leftSetpoint), ff.calculate(rightSetpoint));
      CatzRobotTracker.getInstance().setFlywheelAccelerating(!atGoal() || isDrawingHighCurrent());
    } else if (speed == TargetSpeed.IDLE) {
      CatzRobotTracker.getInstance().setFlywheelAccelerating(false);
      io.stop();
    }

    Logger.recordOutput("Flywheels/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoalLeftRpm", leftGoal);
    Logger.recordOutput("Flywheels/GoalRightRpm", rightGoal);
  }

  //-----------------------------------------------------------------------------------------
  //    Flywheel Misc Methods
  //-----------------------------------------------------------------------------------------
  /** Runs flywheels at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    setSpeed(TargetSpeed.CHARACTERIZING);
    io.runCharacterizationLeft(input);
    io.runCharacterizationRight(input);
  }

  /** Get characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  /** Get if velocity profile has ended */
  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return speed == TargetSpeed.IDLE
        || (leftProfile.getCurrentSetpoint() == speed.getLeftTargetSpeed()
            && rightProfile.getCurrentSetpoint() == speed.getRightTargetSpeed());
  }

  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  }

  //-----------------------------------------------------------------------------------------
  //    Command Flywheel State Access methods
  //-----------------------------------------------------------------------------------------
  /** Set the current goal of the flywheel */
  private void setSpeed(TargetSpeed goal) {
    if (goal ==TargetSpeed.CHARACTERIZING || goal == TargetSpeed.IDLE) {
      wasClosedLoop = closedLoop;
      closedLoop = false;
      this.speed = goal;
      return; // Don't set a goal
    }
    // If not already controlling to requested goal
    // set closed loop false
    closedLoop = this.speed == goal;
    // Enable close loop
    if (!closedLoop) {
      leftProfile.setGoal(goal.getLeftTargetSpeed(), inputs.leftVelocityRpm);
      rightProfile.setGoal(goal.getRightTargetSpeed(), inputs.rightVelocityRpm);
      closedLoop = true;
    }
    this.speed = goal;
  }

  public Command shootCommand() {
    return startEnd(() -> setSpeed(TargetSpeed.SHOOT), () -> setSpeed(TargetSpeed.IDLE))
        .withName("Flywheels Shoot");
  }

  public Command intakeCommand() {
    return startEnd(() -> setSpeed(TargetSpeed.INTAKE), () -> setSpeed(TargetSpeed.IDLE))
        .withName("Flywheels Intake");
  }

  public Command ejectCommand() {
    return startEnd(() -> setSpeed(TargetSpeed.EJECT), () -> setSpeed(TargetSpeed.IDLE))
        .withName("Flywheels Eject");
  }

  public Command hoardCommand() {
    return startEnd(() -> setSpeed(TargetSpeed.POOP), () -> setSpeed(TargetSpeed.IDLE))
          .withName("Flywheels Poop");
  }

  public Command autoHoardCommand() {
    return startEnd(() -> setSpeed(TargetSpeed.HOARD), () -> setSpeed(TargetSpeed.IDLE))
        .withName("Flywheels Super Poop");
  }
}
