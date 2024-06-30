// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotEnviroment;
import frc.robot.commands.AutoSpecifiedCmds;
import frc.robot.commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIO;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIOLimeLight;
import frc.robot.subsystems.LEDs.CatzLED;
import frc.robot.subsystems.Shooter.ShooterFeeder.CatzShooterFeeder;
import frc.robot.subsystems.elevator.CatzElevator;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {

  private static CatzDrivetrain   drive        = new CatzDrivetrain();
  private static CatzElevator     elevator     = new CatzElevator();
  private static CatzShooterFeeder shooterFeeder = new CatzShooterFeeder();
  private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  // private static CatzVision       vision       = new CatzVision(new VisionIO[] {
  //                                                             new VisionIOLimeLight("limelight-udon"),    //index 0 left
  //                                                             new VisionIOLimeLight("limelight-soba"),    //index 1 right
  //                                                             new VisionIOLimeLight("limelight-ramen") 
  //                                                             });   //index 2 turret)

  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  private final LoggedDashboardChooser<AllianceColor> allianceChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");

  private AutoSpecifiedCmds autoSpecifiedCmds = new AutoSpecifiedCmds(this);


  public RobotContainer() {
    allianceChooser.addDefaultOption("blue", AllianceColor.Blue);
    allianceChooser.addOption("Red", AllianceColor.Red);

    configureBindings();

    // Endgame alert triggers
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
                .finallyDo(() -> CatzLED.getInstance().endgameAlert = false));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            controllerRumbleCommand()
                .withTimeout(0.2)
                .andThen(Commands.waitSeconds(0.1))
                .repeatedly()
                .withTimeout(0.9) // Rumble three times
                .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
                .finallyDo(() -> CatzLED.getInstance().endgameAlert = false));
  }

  private void configureBindings() {
    // Default commands
    defaultCommands();
    
    // DriveCommands
    commandsDrive();
    
    // AuxCommands
    commandsAux();

  }

  private void commandsDrive() {

  }

  private void commandsAux() {

  }

  private void defaultCommands() {
    // drive.setDefaultCommand(new TeleopDriveCmd(()->xboxDrv.getLeftX(), 
    //                                            ()->xboxDrv.getLeftY(), 
    //                                            ()->xboxDrv.getRightX(), 
    //                                            ()->xboxDrv.a().getAsBoolean(), drive));
  }

  /** Creates a controller rumble command with specified rumble and controllers */
  private Command controllerRumbleCommand() {
    return Commands.startEnd(
        () -> {
          xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 1.0);
          xboxAux.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          xboxDrv.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          xboxAux.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }

  /** Updates dashboard data. */ //TODO if needed add values here
  public void updateDashboardOutputs() {

  }

  /** Updates the alerts. */
  public void updateAlerts() {

  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(xboxDrv.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort()));
  }

  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }



  public Command getAutonomousCommand() {
    return autoSpecifiedCmds.getCommand();
  }
}
