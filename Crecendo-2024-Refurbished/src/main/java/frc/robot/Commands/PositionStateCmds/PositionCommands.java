// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PositionStateCmds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakePivot.CatzIntakePivot;
import frc.robot.subsystems.elevator.CatzElevator;
import frc.robot.subsystems.elevator.CatzElevator.ElevatorPosition;

/** Add your docs here. */
public class PositionCommands {

    public static PositionStateCommand previousPositionStateCommand = PositionStateCommand.STOW;
    public static enum PositionStateCommand {
      STOW,
      AUTO_AIM,
      SCORE_AMP,
      INTAKE_GROUND,
      INTAKE_SOURCE
    }

    static boolean isIntakeInDanger;
    static boolean isIntakeInFinalSetpoint;
    public static Command Stow(RobotContainer container) {
        CatzElevator elevator = container.getCatzElevator();
        CatzIntakePivot intakePivot = container.getCatzIntakePivot();

        return new FunctionalCommand(
            ()->{
                    switch(previousPositionStateCommand) {
                        case INTAKE_SOURCE:
                        case SCORE_AMP:
                        // Move intake to upright position
                        elevator.setTargetPosition(ElevatorPosition.STOW);
                        isIntakeInDanger = true;
                        break;
                
                        case INTAKE_GROUND:
                        case AUTO_AIM:
                        elevator.setTargetPosition(ElevatorPosition.STOW);
                        isIntakeInDanger = false;
                        // Move intake to final position
                        isIntakeInFinalSetpoint = true;
                        case STOW:
                        break;
                    }
                                    
                    // Init any remaining flags
                    isIntakeInFinalSetpoint = false;
                }, 

            ()->{    
                    if(isIntakeInDanger == true) {
                        if(elevator.getElevatorPosition() > 90.0) {
                        // Move intake into position
                        }
                    }
            
                    if(isIntakeInDanger == false && isIntakeInFinalSetpoint == false) {
                    // Move intake into Final Position
                    }
                }, 
            interrupted -> elevator.endCharacterization(), 
            ()-> false,
             elevator, intakePivot);
    }

    public Command StowSequence() {
        return new SequentialCommandGroup(
        );
    };

    private boolean isIntakeInDanger() {
        boolean c_isIntakeInDanger = false;
        switch(PositionCommands.previousPositionStateCommand) {
            case INTAKE_SOURCE:
            case SCORE_AMP:
              c_isIntakeInDanger = true;
            break;
      
            case INTAKE_GROUND:
            case AUTO_AIM:
            case STOW:
              c_isIntakeInDanger = false;
            break;
        }
        return c_isIntakeInDanger;
    }


    public static void logPositionInteractionVariables() {

    }

}
