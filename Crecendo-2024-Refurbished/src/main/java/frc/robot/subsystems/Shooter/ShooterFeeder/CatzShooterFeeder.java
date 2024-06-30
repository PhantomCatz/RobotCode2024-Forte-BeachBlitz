// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterFeeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class CatzShooterFeeder extends SubsystemBase {

  // HardWare Implementations
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private ShooterFeederState currentShooterFeederState = ShooterFeederState.DISABLED;
  private ShooterFeederState previousShooterFeederState = ShooterFeederState.DISABLED;
  private AdjustState        determinedAdjustState = AdjustState.UNDETERMINED;

  // Flag Definement
  private boolean isAdjustStateDetermined;
  private boolean hasNoteClearedIntake;

  public enum ShooterFeederState{
    TO_SHOOTER,
    TO_INTAKE,
    DISABLED
  }

  private enum AdjustState{
    FWD,
    BCK,
    UNDETERMINED
  }
  
  /** Creates a new CatzShooterFeeder. */
  public CatzShooterFeeder() {
     if(ShooterConstants.isShooterDisabledFeeder) {
      io = new FeederIONull();
      System.out.println("Feeder Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new FeederIOReal();
          System.out.println("Feeder Configured for Real");
          break;

        case REPLAY:
          io = new FeederIOReal() {};
          System.out.println("Feeder Configured for Replayed simulation");
          break;

        case SIM:
          io = new FeederIOSim();
          System.out.println("Feeder Configured for WPILIB simulation");
          break;

        default:
          io = null;
          System.out.println("Feeder Unconfigured");
          break;
      }
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ShooterFeeder/inputs ", inputs); 

    // Feeder State Change Init
    if(currentShooterFeederState != previousShooterFeederState) {
      switch(currentShooterFeederState) {
        case TO_SHOOTER:
          handleToShooterInit();
        break;
        case TO_INTAKE:
          handleToIntakeInit();
        break;
        case DISABLED:
          io.disabled();
        break;
      }
    }

    // Feeder State Periodic
    switch(currentShooterFeederState) {
      case TO_SHOOTER:
        handleToShooterPeriodic();
      break;
      case TO_INTAKE:
        handleToIntakePeriodic();
      break;
      case DISABLED:
      break;
    }

    previousShooterFeederState = currentShooterFeederState;

  } //-End of feeder Periodic

  /** Feeder TOSHOOTER state on state change */
  private void handleToShooterInit() {
    io.loadFoward();
    isAdjustStateDetermined = false;
    determinedAdjustState = AdjustState.UNDETERMINED;
  }

  /** Feeder TOSHOOTER state Periodic */
  private void handleToShooterPeriodic() {
    // Determine which direction to adjust note to
    if(isAdjustStateDetermined == false) {
      if(inputs.isLoadBeamBreakBroken) { 
        if(inputs.isAdjustBeamBreakBroken) {
          io.fineAdjustBck();
          determinedAdjustState = AdjustState.BCK;
          isAdjustStateDetermined = true; //Logic exit
        } else {
          io.fineAdjustFwd();
          determinedAdjustState = AdjustState.FWD;
          isAdjustStateDetermined = true; //Logic exit
        }
      }
    }

    // Adjust Note
    if(isAdjustStateDetermined == true) {
      if(determinedAdjustState == AdjustState.BCK) {
        if(inputs.isAdjustBeamBreakBroken == false) {
          io.disabled();
        }
      } else {
        if(inputs.isAdjustBeamBreakBroken == true) {
          io.disabled();
        }
      }
    }
  }

  /** Feeder TOINTAKE state state change */
  private void handleToIntakeInit() {
    io.resetLoadEnc();
    io.loadFoward();
    hasNoteClearedIntake = false;
  }

  /** Feeder TOINTAKE state Periodic */
  private void handleToIntakePeriodic() {
    // Measure how far note travels
    if(hasNoteClearedIntake == false) {
      if(inputs.noteMovementUpInches > 2.0) {
        io.loadBackward();
        hasNoteClearedIntake = true; //Logic Exit
      }
    }
  }


  /** Shooter Feeder Access Method for Changing state */
  public void setCurrentShooterFeederState(ShooterFeederState wantedState) {
    currentShooterFeederState = wantedState;
  }
}
