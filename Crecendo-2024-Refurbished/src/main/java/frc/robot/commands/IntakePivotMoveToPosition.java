// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot.CatzIntake;
import frc.robot.subsystems.Intake.IntakePivot.IntakeConstants;

public class IntakePivotMoveToPosition extends Command {
  
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
  
  private CatzIntake intake2; //Holden's change to help get access to get methods from intake subsystem, not sure if optimal

  public IntakePivotMoveToPosition(CatzIntake intake)
  {
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_currentPositionDeg = intake2.getWristAngle();
    positionErrorDeg = m_currentPositionDeg - m_targetPositionDeg;

    // voltage control calculation
    m_ffVolts = calculatePivotFeedFoward(Math.toRadians(m_currentPositionDeg), pivotVelRadPerSec, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculatePivotFeedFoward(double positionRadians, double velocityRadPerSec,
      double accelRadPerSecSquared) {
    double finalFF = PIVOT_FF_kS * Math.signum(velocityRadPerSec)
                  + PIVOT_FF_kG * Math.cos(positionRadians)
                  + PIVOT_FF_kV * velocityRadPerSec
                  + PIVOT_FF_kA * accelRadPerSecSquared;
    return finalFF;
  };

}
