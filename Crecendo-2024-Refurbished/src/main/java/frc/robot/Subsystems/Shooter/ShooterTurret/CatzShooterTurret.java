// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.ShooterTurret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;


public class CatzShooterTurret extends SubsystemBase {

  // Hardware IO declaration
  private final TurretIO io;
  private final TurretIOinput

  
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
