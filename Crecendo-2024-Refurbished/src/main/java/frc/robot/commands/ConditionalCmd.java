// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ConditionalCmd extends Command
{
  // private final Command onTrueCommand;
  // private final Command onFalseCommand;
  // private final Boolean conditionBoolean;
  // private Command selectedCommand;

  /**
   * @param onTrue    the command to run if the condition is true
   * @param onFalse   the command to run if the condition is false
   * @param condition the condition to determine which command to run
   */
  // public ConditionalCommand(Command onTrue, Command onFalse, Boolean condition) {
  //     onTrueCommand = requireNonNullParam(onTrue, "onTrue", "ConditionalCommand");
  //     onFalseCommand = requireNonNullParam(onFalse, "onFalse", "ConditionalCommand");

  //     CommandScheduler.getInstance().registerComposedCommands(onTrue, onFalse);
  // }

  // @Override
  // public void initialize()
  // {
  //   onTrueCommand = onTrue;

  //   if (conditionBoolean)
  //   {
  //     selectedCommand = onTrueCommand;
  //   }
    
  //   else
  //   {
  //       selectedCommand = onFalseCommand;
  //   }

  //   selectedCommand.initialize();
  // }

  // @Override
  // public void execute()
  // {
  //   selectedCommand.execute();
  // }

  // @Override
  // public void end(boolean interrupted)
  // {
  //   selectedCommand.end(interrupted);
  // }

  // @Override
  // public boolean isFinished()
  // {
  //   return selectedCommand.isFinished();
  // }

  // @Override
  //   public void initSendable(SendableBuilder builder) {
  //       super.initSendable(builder);
  //       builder.addStringProperty("onTrue", onTrueCommand::getName, null);
  //       builder.addStringProperty("onFalse", onFalseCommand::getName, null);
  //       builder.addStringProperty(
  //               "selected",
  //               () -> {
  //                   if (selectedCommand == null) {
  //                       return "null";
  //                   } else {
  //                       return selectedCommand.getName();
  //                   }
  //               },
  //               null);
  //   }
}
