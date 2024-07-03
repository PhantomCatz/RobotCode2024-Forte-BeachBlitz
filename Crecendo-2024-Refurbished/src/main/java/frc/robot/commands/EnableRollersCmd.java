// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakePivot.CatzIntake;
import frc.robot.subsystems.Intake.IntakePivot.IntakeIOReal;

public class EnableRollersCmd extends Command
{
  private boolean loadBeamBrkState;
  IntakeIOReal intakeioreal = new IntakeIOReal(); //temporary solution to get access to beam break digital input
                                                  //for loadBeamBreak.get()

  public EnableRollersCmd(CatzIntake intake)
  {
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute()
  {
    loadBeamBrkState = intakeioreal.loadBeamBreak.get();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished()
  {
    return loadBeamBrkState;
  }
}
