// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.commands.CmdRollerIn;
import frc.robot.subsystems.Intake.IntakeRollers.CatzIntake;
import frc.robot.subsystems.Intake.IntakeRollers.IntakeIO.IntakeIOInputs;

public class RobotContainer {

  private static CatzIntake intake = new CatzIntake();

  private CommandXboxController xboxDrv = new CommandXboxController(OIConstants.XBOX_DRV_PORT);

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings() {
    //DriveCommands
    commandsDrive();
  }

  private void commandsDrive() {
    //Intake Rollers
    xboxDrv.leftBumper().whileTrue(intake.cmdRollerIn());
    xboxDrv.rightBumper().whileTrue(intake.cmdRollerOut());
    xboxDrv.a().onTrue(intake.cmdStopRollersAfterTimeOut());

    xboxDrv.b().onTrue(new CmdRollerIn(intake));
  }
}
