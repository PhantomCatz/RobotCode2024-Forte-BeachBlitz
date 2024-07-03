// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

public class CatzIntake extends SubsystemBase {
  /** Creates a new CatzIntake. */
  private CatzIntake()
  {
    switch (CatzConstants.hardwareMode) {
      case REAL:
        // io = new IntakeIOReal();
        System.out.println("Intake Configured for Real");
        break;

      case REPLAY:
        // io = new IntakeIOReal() {};
        System.out.println("Intake Configured for Replayed simulation");
        break;

      case SIM:
      default:
        // io = null;
        System.out.println("Intake Unconfigured");
        break;
    }
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("intake/inputs", inputs);

    // // collect ff variables and pid variables
    // IntakeConstants.m_currentPositionDeg = calcWristAngleDeg();
    // IntakeConstants.positionErrorDeg = IntakeConstants.m_currentPositionDeg - IntakeConstants.m_targetPositionDeg;

    // // voltage control calculation
    // IntakeConstants.m_ffVolts = calculatePivotFeedFoward(Math.toRadians(m_currentPositionDeg), pivotVelRadPerSec, 0);
  }
}
