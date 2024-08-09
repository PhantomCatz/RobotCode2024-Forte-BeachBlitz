// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PositionStateCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.PositionStateCmds.PositionStateLogging.PositionStateCommand;
import frc.robot.Subsystems.Elevator.CatzElevator;
import frc.robot.Subsystems.Elevator.CatzElevator.ElevatorPosition;

public class SCORE_AMP extends Command {

  private CatzElevator m_elevator;

  private boolean isIntakeInDanger;
  private boolean isIntakeSetpointedToFinalPos;

  /** Creates a new SCORE_AMP. */
  public SCORE_AMP(CatzElevator elevator) {
    this.m_elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(PositionStateLogging.previousPositionStateCommand) {
      case SCORE_AMP:
      case INTAKE_SOURCE:
      case AUTO_AIM:
        m_elevator.setTargetPosition(ElevatorPosition.SCORE_AMP);
        isIntakeInDanger = false;
        //Move Intake to final position
        isIntakeSetpointedToFinalPos = true;
      break;

      case INTAKE_GROUND:
      case STOW:
        // Intake Move to upright position
        m_elevator.setTargetPosition(ElevatorPosition.WAIT);
        isIntakeInDanger = true;
      break;
    }

    // Init any remaining flags
    isIntakeSetpointedToFinalPos = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run Intake Danger Checks if Applicable
    if(isIntakeInDanger == true) {
      if(true) { // Intake has passed into Upright position
        m_elevator.setTargetPosition(ElevatorPosition.SCORE_AMP);
        isIntakeInDanger = false;
      } 
    }

    // Run Intake w/o Danger check
    if(isIntakeInDanger == false && isIntakeSetpointedToFinalPos == false) {
      // set Intake to Score Amp 
      isIntakeSetpointedToFinalPos = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PositionStateLogging.previousPositionStateCommand = PositionStateCommand.SCORE_AMP;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
