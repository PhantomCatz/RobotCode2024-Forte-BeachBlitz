// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeRollers.CatzIntake;

public class CmdRollerIn extends Command {
  private final CatzIntake m_intake;

  public CmdRollerIn(CatzIntake intake) {
    m_intake = intake;
    
    addRequirements(intake);
  }
  
  @Override
  public void initialize()
  {
    m_intake.setRollerIn();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted)
  {
    m_intake.setRollerOff();
  }

  @Override
  public boolean isFinished() {
    return m_intake.getBeamBrkState();
  }
}
