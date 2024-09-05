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
import frc.robot.autonomous.CatzAutoRoutines;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotSenario;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TeleopDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.Alert.AlertType;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.CatzVision;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIO;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIOLimeLight;
import frc.robot.subsystems.IntakeRollers.CatzIntakeRollers;
import frc.robot.subsystems.LEDs.CatzLED;
import frc.robot.subsystems.Shooter.ShooterFeeder.CatzShooterFeeder;
import frc.robot.subsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.subsystems.SuperSubsystem.CatzSuperSubsystem;
import frc.robot.subsystems.SuperSubsystem.CatzSuperSubsystem.SuperstructureState;
import frc.robot.subsystems.SuperSubsystem.IntakePivot.CatzIntakePivot;
import frc.robot.subsystems.SuperSubsystem.ShooterPivot.CatzShooterPivot;
import frc.robot.subsystems.SuperSubsystem.ShooterTurret.CatzShooterTurret;
import frc.robot.subsystems.SuperSubsystem.elevator.CatzElevator;

public class RobotContainer {

  // Subsystem Declaration
  private static CatzDrivetrain   drive                = new CatzDrivetrain();
  private static CatzShooterFeeder shooterFeeder       = new CatzShooterFeeder();
  private static CatzShooterFlywheels shooterFlywheels = new CatzShooterFlywheels();
  private static CatzIntakeRollers rollers             = new CatzIntakeRollers();

  // Superstructure member subsystem declaration
  private static CatzElevator     elevator             = new CatzElevator();
  private static CatzShooterTurret turret              = new CatzShooterTurret();
  private static CatzShooterPivot  shooterPivot        = new CatzShooterPivot();
  private static CatzIntakePivot   intakePivot         = new CatzIntakePivot();
  private static CatzSuperSubsystem superstructure = new CatzSuperSubsystem(elevator, turret, shooterPivot, intakePivot);

  // Assistance Subsystem declaration
  private static CatzLED          led = CatzLED.getInstance();
  private static CatzRobotTracker robotTracker = CatzRobotTracker.getInstance();
  private static CatzVision       vision       = new CatzVision(new VisionIO[] {
                                                              new VisionIOLimeLight("limelight-udon"),    //index 0 left
                                                              new VisionIOLimeLight("limelight-soba"),    //index 1 right
                                                              new VisionIOLimeLight("limelight-ramen")    //index 2 turret)
                                                              });
  // Drive Controller Declaration
  private CommandXboxController xboxDrv = new CommandXboxController(0);
  private CommandXboxController xboxAux = new CommandXboxController(1);

  // Alert Declaration
  private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
  private final LoggedDashboardNumber endgameAlert1 = new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 = new LoggedDashboardNumber("Endgame Alert #2", 15.0);

  // Auto Declaration
  private CatzAutoRoutines auto = new CatzAutoRoutines(this);


  public RobotContainer() {
    // Drive And Aux Command Mapping
    configureBindings();

    // Endgame alert triggers
    new Trigger(
            () -> DriverStation.isTeleopEnabled()
                  && DriverStation.getMatchTime() > 0.0
                  && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get())
    ).onTrue(
        controllerRumbleCommand()
            .withTimeout(0.5)
            .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
            .finallyDo(() -> CatzLED.getInstance().endgameAlert = false)
    );
    new Trigger(
            () -> DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get())
    ).onTrue(
        controllerRumbleCommand()
            .withTimeout(0.2)
            .andThen(Commands.waitSeconds(0.1))
            .repeatedly()
            .withTimeout(0.9) // Rumble three times
            .beforeStarting(() -> CatzLED.getInstance().endgameAlert = true)
            .finallyDo(() -> CatzLED.getInstance().endgameAlert = false)
    );
  }

  //---------------------------------------------------------------------------
  //
  //  Button mapping to commands
  //
  //---------------------------------------------------------------------------
  private void configureBindings() { // TODO organize by function
  
    xboxAux.rightBumper().whileTrue(rollers.setRollersIn());
    xboxAux.leftBumper().whileTrue(rollers.setRollersOut());

    xboxAux.y().onTrue(superstructure.setSuperStructureState(SuperstructureState.AUTO_AIM));

    xboxAux.a().onTrue(superstructure.setSuperStructureState(SuperstructureState.STOW));
    xboxAux.x().onTrue(superstructure.setSuperStructureState(SuperstructureState.INTAKE_GROUND));
    xboxAux.b().onTrue(superstructure.setSuperStructureState(SuperstructureState.SCORE_AMP));


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

  //---------------------------------------------------------------------------
  //
  //  Misc methods
  //
  //---------------------------------------------------------------------------
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
            || !DriverStation.getJoystickIsXbox(xboxDrv.getHID().getPort())
    );
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(xboxAux.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(xboxAux.getHID().getPort())
    );
  }

  //---------------------------------------------------------------------------
  //
  //      Subsystem getters
  //
  //---------------------------------------------------------------------------
  public CatzDrivetrain getCatzDrivetrain() {
    return drive;
  }

  public CatzSuperSubsystem getCatzSuperstructure() {
    return superstructure;
  }

  public CatzElevator getCatzElevator() {
    return elevator;
  }

  public CatzShooterFeeder getCatzShooterFeeder() {
    return shooterFeeder;
  }

  public CatzShooterFlywheels getCatzShooterFlywheels() {
    return shooterFlywheels;
  }

  public CatzIntakeRollers getCatzIntakeRollers() {
    return rollers;
  }

  public CatzIntakePivot getCatzIntakePivot() {
    return intakePivot;
  }



  public Command getAutonomousCommand() {
    return auto.getCommand();
  }
}
