// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterTurret.TurretConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CatzConstants;
import frc.robot.Utilities.AutoAimingParametersUtil;
import lombok.RequiredArgsConstructor;


public class CatzShooterTurret {

  // Hardware IO declaration
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // Misc variables
  private TurretPosition currentMotionType = TurretPosition.HOME;
  private static double targetTurretPosition = 0.0;
  private static double manualPwr = 0.0;


  // State amchine
  @RequiredArgsConstructor
  public enum TurretPosition {
    AUTO_AIM(()-> AutoAimingParametersUtil.getAutoAimSpeakerParemeters()
                                          .turretHeading()
                                          .getDegrees()), 
    HOME(()->0.0),
    TEST_POSITION(()-> 90.0),
    IDLE(()->targetTurretPosition),
    MANUAL(()->manualPwr);

    private final DoubleSupplier motionType;

    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }
  
  /** Creates a new CatzShooterTurret. */
  public CatzShooterTurret() {
         if(TurretConstants.isShooterTurretDisabled) {
      io = new TurretIONull();
      System.out.println("Feeder Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new TurretIOReal();
          System.out.println("Turret Configured for Real");
        break;
        case REPLAY:
          io = new TurretIOReal() {};
          System.out.println("Turret Configured for Replayed simulation");
        break;
        case SIM:
          io = new TurretIOSim();
          System.out.println("Turret Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Turret Unconfigured");
        break;
      }
    }
  }


  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret/", inputs);

    if(DriverStation.isDisabled()) {
      io.runPercentOutput(0.0);
    } else {
      io.runSetpointDegrees(inputs.positionDegrees, currentMotionType.getTargetMotionPosition());
    }

  }

  //-----------------------------------------------------------------------------------------
  //
  //    Turret Misc Methods
  //
  //-----------------------------------------------------------------------------------------

  public double getTurretPosition() {
    return inputs.positionDegrees;
  }


  //-----------------------------------------------------------------------------------------
  //
  //    State Setting method
  //
  //-----------------------------------------------------------------------------------------
  public void setTargetPosition(TurretPosition type) {
    currentMotionType = type;
  }
}
