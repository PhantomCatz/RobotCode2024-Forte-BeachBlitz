// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.commands.AutoSpecifiedCmds;
import frc.robot.subsystems.Intake.IntakePivot.CatzIntake;

public class RobotContainer {

  private static CatzIntake intake = new CatzIntake();

  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  private static LoggedDashboardChooser<AllianceColor> allianceChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

  private AutoSpecifiedCmds autoSpecifiedCmds = new AutoSpecifiedCmds(this);

  public RobotContainer() {
    // allianceChooser.addDefaultOption("Blue", AllianceColor.Blue);
    // allianceChooser.addOption("Red", AllianceColor.Red);

    configureBindings();
  }

  private void configureBindings() {
    //DriveCommands
    commandsDrive();

    //AuxCommands
    // commandsAux();
  }

  private void commandsDrive() {
    //Intake Rollers
    xboxDrv.leftBumper().onTrue(intake.cmdRollerIn());
    xboxDrv.rightBumper().onTrue(intake.cmdRollerOut());
    xboxDrv.leftBumper().and(xboxDrv.rightBumper()).onTrue(intake.cmdRollerOff());
    xboxDrv.a().onTrue(intake.cmdStopRollersAfterTimeOut());

    //Intake Pivot
    // xboxDrv.b().onTrue(intake.cmdSetIntakePivotGround());
  }

  private void commandsAux() {}

  public Command getAutonomousCommand()
  {
    return autoSpecifiedCmds.getCommand();
  }
}
