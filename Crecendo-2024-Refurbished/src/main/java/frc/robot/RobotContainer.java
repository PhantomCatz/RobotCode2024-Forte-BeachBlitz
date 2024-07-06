// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.subsystems.Intake.IntakePivot.CatzIntake;

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
    xboxDrv.leftBumper().onTrue(intake.cmdRollerIn());
    xboxDrv.rightBumper().onTrue(intake.cmdRollerOut());
    xboxDrv.leftBumper().and(xboxDrv.rightBumper()).onTrue(intake.cmdRollerOff());
    xboxDrv.a().onTrue(intake.cmdStopRollersAfterTimeOut());
  }
}
