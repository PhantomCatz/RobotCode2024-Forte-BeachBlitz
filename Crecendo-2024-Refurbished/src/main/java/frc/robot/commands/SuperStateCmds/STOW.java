// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SuperStateCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.SuperstructureCommandLogger;
import frc.robot.SuperstructureCommandLogger.SuperStateCommand;
import frc.robot.subsystems.elevator.CatzElevator;
import frc.robot.subsystems.elevator.CatzElevator.ElevatorState;

public class STOW extends Command {

  private CatzElevator m_elevator;

  private boolean isIntakeInDanger;
  private boolean isIntakeInFinalSetpoint;

  /** Creates a new STOW. */
  public STOW(RobotContainer container) {
    m_elevator = container.getCatzElevator();
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(SuperstructureCommandLogger.previousSuperStateCommand) {
      case INTAKE_SOURCE:
      case SCORE_AMP:
        // Move intake to upright position
        m_elevator.setElevatorState(ElevatorState.STOW);
        isIntakeInDanger = true;
      break;

      case INTAKE_GROUND:
      case AUTO_AIM:
        m_elevator.setElevatorState(ElevatorState.STOW);
        isIntakeInDanger = false;
        // Move intake to final position
        isIntakeInFinalSetpoint = true;
      case STOW:
      break;
    }

    // Init any remaining flags
    isIntakeInFinalSetpoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isIntakeInDanger == true) {
      if(m_elevator.getElevatorPosition() > 90.0) {
        // Move intake into position
      }
    }

    if(isIntakeInDanger == false && isIntakeInFinalSetpoint == false) {
      // Move intake into Final Position
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SuperstructureCommandLogger.previousSuperStateCommand = SuperStateCommand.STOW;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
