// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterFeeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CatzShooterFeeder extends SubsystemBase {

  private ShooterFeederState currentShooterFeederState;
  private ShooterFeederState previousShooterFeederState;

  public enum ShooterFeederState{
    TO_SHOOTER,
    TO_INTAKE
  }
  
  /** Creates a new CatzShooterFeeder. */
  public CatzShooterFeeder() {}

  @Override
  public void periodic() {

    if(currentShooterFeederState != previousShooterFeederState) {
      switch(currentShooterFeederState) {
        case TO_SHOOTER:
          handleToShooterInit();
        break;
        case TO_INTAKE:
          handleToIntakeInit();
        break;
      }
    }

    switch(currentShooterFeederState) {
      case TO_SHOOTER:
        handleToShooterExecute();
      break;
      case TO_INTAKE:
        handleToIntakeExecute();
      break;
    }



  }

  private void handleToShooterInit() {
    
  }

  private void handleToShooterExecute() {

  }

  private void handleToIntakeInit() {

  }

  private void handleToIntakeExecute() {

  }

  public void setCurrentShooterFeederState(ShooterFeederState wantedState) {
    currentShooterFeederState = wantedState;
  }
}
