// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.ShooterPivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.ShooterTurret.CatzShooterTurret.TurretPosition;
import lombok.RequiredArgsConstructor;

public class CatzShooterPivot {
  
  // Hardware IO Declaration


  // Misc variables
  private ShooterPivotMotionType currentMotionType;
  private double targetShooterPivotPosition = 0.0;
  private double manualPwr = 0.0;


  //State machine
  @RequiredArgsConstructor
  public enum ShooterPivotMotionType {
    AUTO_AIM(()->0.0), // TODO add auto aim parameters
    HOME(()->0.0);

    private final DoubleSupplier motionType;
    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }


  public CatzShooterPivot() {}

  public void periodic() {
    // This method will be called once per scheduler run
  }


  //-----------------------------------------------------------------------------------------
  //
  //    State Setting method
  //
  //-----------------------------------------------------------------------------------------
  public void setTargetMotionMethod(ShooterPivotMotionType type) {
    currentMotionType = type;
  }
}
