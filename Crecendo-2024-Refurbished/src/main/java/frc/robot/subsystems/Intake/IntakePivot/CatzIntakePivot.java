// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.Intake.IntakePivot.IntakePivotConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.Intake.IntakePivot.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.elevator.CatzElevator.ElevatorState;
import frc.robot.utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class CatzIntakePivot extends SubsystemBase {

  // Hardware Implementation
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Intake Statemachine variables
  private IntakePivotState m_intakePivotState;

  @RequiredArgsConstructor
  public static enum IntakePivotState {
    SCORE_AMP(new LoggedTunableNumber("Intake/Pivot Score Amp", 90.0)),
    PICKUP_SOURCE(new LoggedTunableNumber("Intake/Pivot Pickup Souce", 90.0)),
    STOW(() -> 0.0),
    WAIT(()-> targetDegree);
    
    private final DoubleSupplier intakePivotSetpointSupplier;

    private double getTargetDegree() {
      return intakePivotSetpointSupplier.getAsDouble();
    }
  }

  //Intake Pivot Class Variables
  private static double targetDegree = 0.0;



  /** Creates a new CatzIntake. */
  private CatzIntakePivot() {
    if(IntakePivotConstants.isIntakePivotDisabled) {
      io = new IntakeIONull();
      System.out.println("Intake Pivot Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new IntakeIOReal();
          System.out.println("Intake Configured for Real");
          break;

        case REPLAY:
          io = new IntakeIOReal() {};
          System.out.println("Intake Configured for Replayed simulation");
          break;

        case SIM:
        default:
          io = null;
          System.out.println("Intake Unconfigured");
          break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);

    // Run Setpoint Control
    if(DriverStation.isDisabled() || m_intakePivotState == null) {
      io.stop();
    } else {
      // Run Softlimit check
      if(getIntakePivotPosition() > ma) {
        io.stop();
      } else {
        // Run state check
        if(m_intakePivotState == IntakePivotState.STOW) {
          // Run Crossbar hit check
          if(getIntakePivotPosition() < 5.0) {
            io.stop();
          } else {
            // Run Setpoint Motion Magic    
            io.runSetpoint(m_intakePivotState.getTargetPositionRotations(), ff.calculate(inputs.velocityRotPerSec));
          }
        } else {
          // Run Setpoint Motion Magic    
          io.runSetpoint(m_intakePivotState.getTargetPositionRotations(), ff.calculate(inputs.velocityRotPerSec));
        }
      }
    }
    
    // // collect ff variables and pid variables
    // IntakeConstants.m_currentPositionDeg = calcWristAngleDeg();
    // IntakeConstants.positionErrorDeg = IntakeConstants.m_currentPositionDeg - IntakeConstants.m_targetPositionDeg;

    // // voltage control calculation
    // IntakeConstants.m_ffVolts = calculatePivotFeedFoward(Math.toRadians(m_currentPositionDeg), pivotVelRadPerSec, 0);
  }

  public double getIntakePivotPosition() {
    return inputs.pivotMotorRev;
  }

  public void setIntakePivotState(IntakePivotState state) {
    m_intakePivotState = state;
  }
}
