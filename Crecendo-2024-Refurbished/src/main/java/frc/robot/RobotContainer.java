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
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.Commands.CatzAutoFactory;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.Subsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.Subsystems.DriveAndRobotOrientation.vision.VisionIO;
import frc.robot.Subsystems.DriveAndRobotOrientation.vision.VisionIOLimeLight;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {

  // Subsystem Declaration
  private static CatzDrivetrain   drive        = new CatzDrivetrain();
  
  //xbox declaration
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  // Alert Declaration
  private final Alert disconnectedAlertDrive = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert disconnectedAlertAux = new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  // Auto Declaration
  private CatzAutoFactory auto = new CatzAutoFactory(this);


  public RobotContainer() {
    // Drive And Aux Command Mapping
    configureBindings();
  }

  private void configureBindings() { // TODO organize by function
    drive.setDefaultCommand(new TeleopDriveCmd(() -> xboxDrv.getLeftX(), 
                                               () -> xboxDrv.getLeftY(), 
                                               () -> xboxDrv.getRightX(), 
                                               () -> xboxDrv.a().getAsBoolean(), drive)); // TODO changes this to be in the subsystem rather than the cmd
    //TODO add triggers to put default as priority    
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
    disconnectedAlertDrive.set(
        !DriverStation.isJoystickConnected(xboxDrv.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort())
    );
    disconnectedAlertAux.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort())
    );
    disconnectedAlertAux.setAlertOnloop(true, 1.0, 10.0);
  }

  //---------------------------------------------------------------------------
  //      Subsystem getters
  //---------------------------------------------------------------------------
  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }

  public Command getAutonomousCommand() {
    return auto.getCommand();
  }
}
