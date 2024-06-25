// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SuperStateCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperstructureCommandLogger;
import frc.robot.SuperstructureCommandLogger.SUPERSTATE_COMMAND;
import frc.robot.subsystems.elevator.CatzElevator;
import frc.robot.subsystems.elevator.CatzElevator.ElevatorState;

public class SCORE_AMP extends Command {

  private CatzElevator m_elevator;

  private boolean isElevatorInScoreAmp;

  /** Creates a new SCORE_AMP. */
  public SCORE_AMP(CatzElevator elevator) {
    this.m_elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(SuperstructureCommandLogger.previousSuperStateCommand) {
      case SCORE_AMP:

      case STOW:
      // Intake Move to upright position
      m_elevator.setElevatorState(ElevatorState.WAIT);
      default:
      break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(true) { // Intake has passed into Upright position
      m_elevator.setElevatorState(ElevatorState.SCORE_AMP);
      isElevatorInScoreAmp = true;
    } 

    if(isElevatorInScoreAmp) {
      // set Intake to Score Amp 
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SuperstructureCommandLogger.previousSuperStateCommand = SUPERSTATE_COMMAND.SCORE_AMP;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
