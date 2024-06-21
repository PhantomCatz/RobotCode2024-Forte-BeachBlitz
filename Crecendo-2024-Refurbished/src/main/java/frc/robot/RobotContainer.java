// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.commands.AutoSpecifiedCmds;
import frc.robot.commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIO;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIOLimeLight;

public class RobotContainer {

  private static CatzDrivetrain   drive        = new CatzDrivetrain();
  //private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  // private static CatzVision       vision       = new CatzVision(new VisionIO[] {
  //                                                             new VisionIOLimeLight("limelight-udon"),    //index 0 left
  //                                                             new VisionIOLimeLight("limelight-soba"),    //index 1 right
  //                                                             new VisionIOLimeLight("limelight-ramen") 
  //                                                             });   //index 2 turret)

  private CommandXboxController xboxDrv = new CommandXboxController(0);

  private static LoggedDashboardChooser<AllianceColor> allianceChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

  private AutoSpecifiedCmds autoSpecifiedCmds = new AutoSpecifiedCmds(this);


  public RobotContainer() {
    allianceChooser.addDefaultOption("blue", AllianceColor.Blue);
    allianceChooser.addOption("Red", AllianceColor.Red);

    configureBindings();
  }

  private void configureBindings() {
    //default commands
    defaultCommands();
    //DriveCommands
    commandsDrive();
    //AuxCommands
    commandsAux();

  }

  private void commandsDrive() {

  }

  private void commandsAux() {

  }

  private void defaultCommands() {
    drive.setDefaultCommand(new TeleopDriveCmd(()->xboxDrv.getLeftX(), 
                                               ()->xboxDrv.getLeftY(), 
                                               ()->xboxDrv.getRightX(), 
                                               ()->xboxDrv.a().getAsBoolean(), drive));
  }

  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }



  public Command getAutonomousCommand() {
    return autoSpecifiedCmds.getCommand();
  }
}
