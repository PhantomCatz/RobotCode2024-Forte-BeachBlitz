// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

public class CatzIntake extends SubsystemBase {
  // -----------------------------------------------------------------------------------------------
  // 
  // Intake IO Block
  // 
  // -----------------------------------------------------------------------------------------------
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // -----------------------------------------------------------------------------------------------
  // 
  // Pivot Closed Loop Processing (PID)
  // 
  // -----------------------------------------------------------------------------------------------
  public static final double PIVOT_PID_kP = 9.00; // 0.044
  public static final double PIVOT_PID_kI = 0.00; // 0.005
  public static final double PIVOT_PID_kD = 0.27;

  // -----------------------------------------------------------------------------------------------
  // 
  // Pivot Open Loop Processing (Feedforward)
  // 
  // -----------------------------------------------------------------------------------------------
  public final double PIVOT_FF_kS = 0.00;
  public final double PIVOT_FF_kG = 0.437;
  public final double PIVOT_FF_kV = 0.00;
  public final double PIVOT_FF_kA = 0.00;

  private double m_ffVolts = 0.0;

  private double m_targetPositionDeg = 0.0;
  private double m_nextTargetPositionDeg = IntakeConstants.INTAKE_NULL_DEG;
  private double m_currentPositionDeg = 0.0;
  private double m_previousTargetPositionDeg = 0.0;

  private boolean isIntakeInScoreAmp;

  private int m_iterationCounter;

  private double positionErrorDeg = 0.0;
  private double pivotVelRadPerSec = 0.0;

  // -----------------------------------------------------------------------------------------------
  // 
  // Control Mode Definitions
  // 
  // -----------------------------------------------------------------------------------------------
  public static enum IntakeControlState {
    AUTO,
    SEMI_MANUAL,
    FULL_MANUAL,
  }

  private static IntakeControlState m_currentIntakeControlState = IntakeControlState.AUTO;
  private double m_pivotManualPwr = 0.0;

  private static boolean m_intakeInPosition = false;
  private boolean m_intermediatePositionReached = false;

  public CatzIntake()
  {
    switch (CatzConstants.currentMode)
    {
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake/inputs", inputs);

    // collect ff variables and pid variables
    m_currentPositionDeg = calcWristAngleDeg();
    positionErrorDeg = m_currentPositionDeg - m_targetPositionDeg;
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Calculation Methods
  // 
  // -------------------------------------------------------------------------------------
  private double calcWristAngleDeg() {
    double wristAngle = inputs.pivotMtrRev / IntakeConstants.INTAKE_PIVOT_MTR_REV_PER_DEG;
    return wristAngle;
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Get Methods
  // 
  // -------------------------------------------------------------------------------------
  public double getWristAngle() {
    return m_currentPositionDeg;
  }

  public boolean getIntakeInPos() {
    return m_intakeInPosition;
  }

  public void setWasIntakeInAmpScoring(boolean set) {
    isIntakeInScoreAmp = set;
  }

  public boolean getIsIntakeInAmpScoring() {
    return isIntakeInScoreAmp;
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Roller Methods
  // 
  // -------------------------------------------------------------------------------------
  public void setRollersIn ()
  {
    io.setRollerPercentOutput(IntakeConstants.ROLLERS_MTR_PWR_IN_GROUND);
  }

  public void setRollersOut ()
  {
    io.setRollerPercentOutput(0.0);
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Roller Commands
  // 
  // -------------------------------------------------------------------------------------
  public Command cmdRollerIn ()
  {
    return runOnce(() -> setRollersIn());
  }

  public Command cmdRollerOut()
  {
    return runOnce(() -> setRollersOut());
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Roller Commands
  // 
  // -------------------------------------------------------------------------------------
  // public Command cmdSetIntakePivotTargetPosition()
  // {

  // }

}
