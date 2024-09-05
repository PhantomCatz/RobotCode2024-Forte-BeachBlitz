// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperSubsystem;

import javax.lang.model.element.ElementKind;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperSubsystem.IntakePivot.CatzIntakePivot;
import frc.robot.subsystems.SuperSubsystem.IntakePivot.CatzIntakePivot.IntakePivotPosition;
import frc.robot.subsystems.SuperSubsystem.ShooterPivot.CatzShooterPivot;
import frc.robot.subsystems.SuperSubsystem.ShooterTurret.CatzShooterTurret;
import frc.robot.subsystems.SuperSubsystem.ShooterTurret.CatzShooterTurret.TurretPosition;
import frc.robot.subsystems.SuperSubsystem.elevator.CatzElevator;
import frc.robot.subsystems.SuperSubsystem.elevator.CatzElevator.ElevatorPosition;

public class CatzSuperSubsystem extends SubsystemBase {

  private static SuperstructureState currentSuperstructureState = SuperstructureState.STOW;
  private static SuperstructureState previousSuperstructureState = SuperstructureState.STOW;
  private static SuperstructureState previousCommandedSuperstructureState = SuperstructureState.STOW;

  public static enum SuperstructureState {
    STOW,
    AUTO_AIM,
    SCORE_AMP,
    INTAKE_GROUND,
    INTAKE_SOURCE
  }

  private final CatzElevator elevator;
  private final CatzShooterTurret turret;
  private final CatzShooterPivot shooterPivot;
  private final CatzIntakePivot intakePivot;


  private boolean isElevatorCausingDanger = false;
  private boolean isTurretCausingDanger = false;
  private boolean isShooterPivotCausingDanger = false;
  private boolean isIntakePivotCausingDanger = false;

  /** Creates a new Superstructure. */
  public CatzSuperSubsystem(CatzElevator elevator, CatzShooterTurret turret, CatzShooterPivot shooterPivot, CatzIntakePivot intakePivot) {
    this.elevator = elevator;
    this.turret = turret;
    this.shooterPivot = shooterPivot;
    this.intakePivot = intakePivot;
  }

  @Override
  public void periodic() {
    //Run Danger Checks for stow
    isTurretCausingDanger = (Math.abs(turret.getTurretPosition()) > 20.0); //Turret turned too far to left or right
    isShooterPivotCausingDanger = false; //TBD change this later //Shooter extended to high
    isElevatorCausingDanger = elevator.getElevatorPositionRotations() > 30.0; // Elevator too high for intake to stow
    isIntakePivotCausingDanger = (intakePivot.getIntakePivotPosition() > 90.0); // Intake not extended out far enough to clear elevator

    // Init SuperstructureState change
    if(currentSuperstructureState != previousSuperstructureState) 
    {
      // Run initializes for each state
      switch(currentSuperstructureState) 
      {
        case STOW:
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.HOME);
          //Set shooter pivot down
          if(!isElevatorCausingDanger && !isTurretCausingDanger && !isShooterPivotCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.STOW);
          } else 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
          }
        break;

        case INTAKE_GROUND:
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.HOME);
          if(!isElevatorCausingDanger && !isTurretCausingDanger && !isShooterPivotCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.PICKUP_GROUND);
          } else 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
          }

        break;

        case SCORE_AMP:
          intakePivot.setIntakePivotState(IntakePivotPosition.SCORE_AMP);
          turret.setTargetPosition(TurretPosition.HOME);
          if(!isIntakePivotCausingDanger) 
          {
            elevator.setTargetPosition(ElevatorPosition.STOW);
          } else 
          {
            elevator.setTargetPosition(ElevatorPosition.SCORE_AMP);
          }
        break;

        case AUTO_AIM:
          elevator.setTargetPosition(ElevatorPosition.STOW);
          turret.setTargetPosition(TurretPosition.AUTO_AIM);
          //Set shooter pivot to autoaim
          intakePivot.setIntakePivotState(IntakePivotPosition.HOLD);
        break;

        default:
      }
      previousCommandedSuperstructureState = previousSuperstructureState;
      previousSuperstructureState = currentSuperstructureState;
    }

    // Periodic Superstructure State Change
    switch(currentSuperstructureState) 
    {
      case STOW:
        if(!isTurretCausingDanger) 
        {
          if(!isElevatorCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.STOW);
          }
        }
      break;

      case INTAKE_GROUND:
        if(!isTurretCausingDanger) 
        {
          if(!isElevatorCausingDanger) 
          {
            intakePivot.setIntakePivotState(IntakePivotPosition.PICKUP_GROUND);
          }
        }
      break;

      case SCORE_AMP:
        if(!isIntakePivotCausingDanger) 
        {
          elevator.setTargetPosition(ElevatorPosition.SCORE_AMP);
        }
      break;

      case AUTO_AIM:
      break;

      default:
    }

    Logger.recordOutput("Superstructure/dangerCheck/elevator", isElevatorCausingDanger);
    Logger.recordOutput("Superstructure/dangerCheck/intake", isIntakePivotCausingDanger);
    Logger.recordOutput("Superstructure/dangerCheck/shooterpivot", isShooterPivotCausingDanger);
    Logger.recordOutput("Superstructure/dangerCheck/Turret", isTurretCausingDanger);


    Logger.recordOutput("Superstructure/currentState", currentSuperstructureState.toString());
    Logger.recordOutput("Superstructure/lastCommandedState", previousCommandedSuperstructureState.toString());
    Logger.recordOutput("Superstructure/previousLoopState", previousSuperstructureState.toString());



    // Run Member Mechanism periodics
    elevator.periodic();
    turret.periodic();
    shooterPivot.periodic();
    intakePivot.periodic();


    previousSuperstructureState = currentSuperstructureState;

  } // - End of Superstructure Periodic


  //-----------------------------------------------------------------------------------------
  //
  //    Superstructure Instance Factory Command Wrapper
  //
  //-----------------------------------------------------------------------------------------
  public Command setSuperStructureState(SuperstructureState newState) {
    return runOnce(() -> currentSuperstructureState = newState);
  }

}
