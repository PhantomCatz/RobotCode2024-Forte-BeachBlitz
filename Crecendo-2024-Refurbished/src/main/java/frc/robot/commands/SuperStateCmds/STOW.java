// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SuperStateCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperstructureCommandLogger;
import frc.robot.SuperstructureCommandLogger.SUPERSTATE_COMMAND;
import frc.robot.subsystems.elevator.CatzElevator;
import frc.robot.subsystems.elevator.CatzElevator.ElevatorState;

public class STOW extends Command {

  private CatzElevator m_elevator;
  /** Creates a new STOW. */
  public STOW(CatzElevator elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(SuperstructureCommandLogger.previousSuperStateCommand) {
      case SCORE_AMP:
      // Move intake to upright position
      m_elevator.setElevatorState(ElevatorState.STOW);
      case STOW:

      default:
      break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_elevator.getElevatorPosition() > 90.0) {
      // Move intake into position
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SuperstructureCommandLogger.previousSuperStateCommand = SUPERSTATE_COMMAND.STOW;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
