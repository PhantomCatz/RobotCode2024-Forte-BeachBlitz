// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  // Intake Pivot
  // 
  // -----------------------------------------------------------------------------------------------

  // -----------------------------------------------------------------------------------------------
  // Pivot Control Loop Processing (PID and FF)
  // planning to delete all commented and keep control loop processing in Command
  // -----------------------------------------------------------------------------------------------
  public static final double PIVOT_PID_kP = 9.00; // 0.044
  public static final double PIVOT_PID_kI = 0.00; // 0.005
  public static final double PIVOT_PID_kD = 0.27;

  // public final double PIVOT_FF_kS = 0.00;
  // public final double PIVOT_FF_kG = 0.437;
  // public final double PIVOT_FF_kV = 0.00;
  // public final double PIVOT_FF_kA = 0.00;

  // private double m_ffVolts = 0.0;

  // private double pivotVelRadPerSec = 0.0;
  // private double positionErrorDeg  = 0.0;

  // -----------------------------------------------------------------------------------------------
  // Position Variables
  // planning to delete all commented and keep in Command instead where logic should be done, not in subsystem periodic
  // -----------------------------------------------------------------------------------------------
  // private double m_targetPositionDeg = 0.0;
  // private double m_nextTargetPositionDeg = IntakeConstants.INTAKE_NULL_DEG;
  private double m_currentPositionDeg = 0.0;
  // private double m_previousTargetPositionDeg = 0.0;

  public IntakeTargetPosition currentTargetPos = null;
  
  // private int m_iterationCounter;

  private boolean isIntakeInScoreAmp;

  // -----------------------------------------------------------------------------------------------
  // 
  // Control Mode Definitions
  // 
  // -----------------------------------------------------------------------------------------------
  public static enum IntakeControlState {
    AUTO,
    SEMI_MANUAL,
    FULL_MANUAL
  }

  public static enum IntakeTargetPosition {
    GROUND,
    SOURCE,
    MIDDLE_TEMP_POSITION
  }

  private static IntakeControlState m_currentIntakeControlState = IntakeControlState.AUTO;
  
  private double m_pivotManualPwr = 0.0;

  private static boolean m_intakeInPosition = false;
  private boolean m_intermediatePositionReached = false;

  // -----------------------------------------------------------------------------------------------
  // 
  // Intake Rollers
  // 
  // -----------------------------------------------------------------------------------------------
  
  // -----------------------------------------------------------------------------------------------
  // Roller States
  // -----------------------------------------------------------------------------------------------
  public static enum IntakeRollerState {
    ROLLERS_IN_SOURCE,
    ROLLERS_IN_GROUND,
    BEAM_BREAK_CHECK,
    NOTE_ADJUST,
    ROLLERS_OUT_EJECT,
    ROLLERS_OUT_SHOOTER_HANDOFF,
    ROLLERS_OFF, 
  }

  public IntakeRollerState m_currentRollerState;


  // -----------------------------------------------------------------------------------------------
  // 
  // CatzIntake
  // 
  // -----------------------------------------------------------------------------------------------
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
    // io.updateInputs(inputs);
    // Logger.processInputs("intake/inputs", inputs);

    // collect ff variables and pid variables
    m_currentPositionDeg = calcWristAngleDeg();

    // io.setIntakePivotPostionRev(m_targetPositionDeg * IntakeConstants.INTAKE_PIVOT_MTR_REV_PER_DEG, m_ffVolts);
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Pivot Methods
  // 
  // -------------------------------------------------------------------------------------
  private double calcWristAngleDeg() {
    double wristAngle = inputs.pivotMtrRev / IntakeConstants.INTAKE_PIVOT_MTR_REV_PER_DEG;
    return wristAngle;
  }
  
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

  public void setIntakePivotTargetPos(IntakeTargetPosition position)
  {
    currentTargetPos = position;
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Pivot Commands
  // 
  // -------------------------------------------------------------------------------------
  public Command cmdSetIntakePivotGround()
  {
    return runOnce(() -> setIntakePivotTargetPos(IntakeTargetPosition.GROUND));
  }

  public Command cmdSetIntakePivotSource()
  {
    return runOnce(() -> setIntakePivotTargetPos(IntakeTargetPosition.SOURCE));
  }

  public Command cmdSetIntakePivotTempMiddle()
  {
    return runOnce(() -> setIntakePivotTargetPos(IntakeTargetPosition.MIDDLE_TEMP_POSITION));
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

  public void setRollersOut()
  {
    io.setRollerPercentOutput(IntakeConstants.ROLLERS_MTR_PWR_OUT_EJECT);
  }

  public void setRollersOff ()
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

  public Command cmdRollerOff()
  {
    return runOnce(() -> setRollersOff());
  }


  public Command cmdStopRollersAfterTimeOut()
  {
    return new SequentialCommandGroup(
      cmdRollerIn(),
      new WaitCommand(2),
      cmdRollerOff()
    );
  }
}
