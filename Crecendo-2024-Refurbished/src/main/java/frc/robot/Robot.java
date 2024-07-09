// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Map;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.google.flatbuffers.Constants;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.RobotEnviroment;
import frc.robot.subsystems.LEDs.CatzLED;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Robot extends LoggedRobot {
  //LED instantiation
  CatzLED led = CatzLED.getInstance();

  private Command m_autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();
  private double teleStart;
  private static double teleElapsedTime = 0.0;

  private static final String batteryNameFile = "/home/lvuser/battery-name.txt";
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double canivoreErrorTimeThreshold = 0.5;
  private static final double lowBatteryVoltage = 11.8;
  private static final double lowBatteryDisabledTime = 1.5;

  private static final String defaultBatteryName = "BAT-0000-000";
  private final StringSubscriber batteryNameSubscriber =
      NetworkTableInstance.getDefault()
          .getStringTopic("/battery_name")
          .subscribe(defaultBatteryName);
  private boolean batteryNameChecked = false;
  private boolean batteryNameWritten = false;

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", AlertType.WARNING);
  private final Alert gcAlert =
      new Alert("Please wait to enable, collecting garbage. 🗑️", AlertType.WARNING);
  private final Alert driverStationDisconnectAlert =
      new Alert("Driverstation is not online, alliance selection will not work", AlertType.ERROR);
  private final Alert fmsDisconnectAlert =
      new Alert("fms is offline, robot cannot compete in match", AlertType.ERROR);

  private RobotContainer m_robotContainer;

  private Optional<Alliance> alliance = Optional.empty();

  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
        case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
        case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
        default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
    }

    // Set up data receivers & replay source
    switch (CatzConstants.hardwareMode) {
        case REAL:
            // Running on a real robot, log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new WPILOGWriter());
            //Logger.addDataReceiver(new WPILOGWriter("E:/Logs"));
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case SIM:
            // Running a physics simulator, log to NT
            //Logger.addDataReceiver(new WPILOGWriter("F:/robotics code projects/loggingfiles/"));
            Logger.addDataReceiver(new NT4Publisher());
            break;

        case REPLAY:
            // Replaying a log, set up replay source
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    // Reset alert timers
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    RobotController.setBrownoutVoltage(6.0);

    System.out.println("Enviorment: " + CatzConstants.robotEnviroment.toString());
    System.out.println("Mode: " + CatzConstants.hardwareMode.toString());
    System.out.println("Type: " + CatzConstants.getRobotType().toString());

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();

    // Print auto duration
    if (m_autonomousCommand != null) {
      if (!m_autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
        }
        autoMessagePrinted = true;
        CatzLED.getInstance().autoFinished = true;
        CatzLED.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
      }

      //TODO add note visualizer stuff
    }

    // Robot container periodic methods
    m_robotContainer.checkControllers();
    m_robotContainer.updateDashboardOutputs();
    m_robotContainer.updateAlerts();

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

    // Low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() <= lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
      CatzLED.getInstance().lowBatteryAlert = true;
    }

    // GC alert
    gcAlert.set(Timer.getFPGATimestamp() < 45.0);

    // Update battery alert
    String batteryName = batteryNameSubscriber.get();
    Logger.recordOutput("BatteryName", batteryName);
    if (CatzConstants.hardwareMode == CatzConstants.HardwareMode.REAL && !batteryName.equals(defaultBatteryName)) {
      // Check for battery alert
      if (!batteryNameChecked) {
        batteryNameChecked = true;
        File file = new File(batteryNameFile);
        if (file.exists()) {
          // Read previous battery name
          String previousBatteryName = "";
          try {
            previousBatteryName =
                new String(Files.readAllBytes(Paths.get(batteryNameFile)), StandardCharsets.UTF_8);
          } catch (IOException e) {
            e.printStackTrace();
          }
          if (previousBatteryName.equals(batteryName)) {
            // Same battery, set alert
            sameBatteryAlert.set(true);
            CatzLED.getInstance().sameBattery = true;
          } else {
            // New battery, delete file
            file.delete();
          }
        }
      }

      // Write battery name if in Competition Mode
      if (!batteryNameWritten && CatzConstants.robotEnviroment == RobotEnviroment.COMPETITION) {
        batteryNameWritten = true;
        try {
          FileWriter fileWriter = new FileWriter(batteryNameFile);
          fileWriter.write(batteryName);
          fileWriter.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Run Driver Station Chooser
    if(DriverStation.isDSAttached()) {
      alliance = DriverStation.getAlliance();
      CatzConstants.choosenAllianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? AllianceColor.Blue : AllianceColor.Red)
              .orElse(AllianceColor.Red);
    } 

    // Driver Station Alerts
    driverStationDisconnectAlert.set(!DriverStation.isDSAttached());
    fmsDisconnectAlert.set(!DriverStation.isFMSAttached());

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    teleStart = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {
    teleElapsedTime = Timer.getFPGATimestamp() - teleStart;
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
