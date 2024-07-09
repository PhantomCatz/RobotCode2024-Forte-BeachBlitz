package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperStateCmds.AUTO_AIM;
import frc.robot.commands.SuperStateCmds.INTAKE_GROUND;
import frc.robot.commands.SuperStateCmds.STOW;
import frc.robot.subsystems.Shooter.ShooterFeeder.CatzShooterFeeder.ShooterFeederState;

/** Place where any sequencing/nonPosition Based robot state logic is held */
public class AutomatedSequenceCmds {
    /** 
     * Runs the Auto Note detect in telop to feed note into shooter
     */
    public static Command NoteDetectIntakeToShooter(RobotContainer container) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new INTAKE_GROUND(), // Until Intake has made it to final ground pos
                Commands.print("Run Rollers")
            ).until(()->true), // Until Intake Rollers have detected note,
            TransferNoteToShooter(container) //Stow is already called in method
        );
    }

    /** 
     * Runs the Auto Note detect in telop to feed note into Intake for amp Scoring
     */
    public static Command NoteDetectIntakeToAmpScoring(RobotContainer container) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new INTAKE_GROUND(), // Until Intake has made it to final ground pos
                Commands.print("Run Rollers")
            ).until(()->true), // Until Intake Rollers have detected note,
            new STOW(container).until(()->true) // Until Intake has stowed 
        );
    }

    /** 
     * Runs shooter to intake note transfer
     */
    public static Command TransferNoteToIntake(RobotContainer container) {
        return new SequentialCommandGroup(
            new STOW(container).until(()->true),
            new ParallelCommandGroup(
                Commands.print("Run Rollers To Intake"),
                container.getCatzShooterFeeder().commandToIntake()
            ).until(()->container.getCatzShooterFeeder()
                                    .isNoteInShooterPosition()) // Until Shooter finalizes note position  
        );
    }

    /** 
     * Runs intake to shooter note transfer
     */
    public static Command TransferNoteToShooter(RobotContainer container) {
        return new SequentialCommandGroup(
            new STOW(container).until(()->true), // Until Intake has stowed 
            new ParallelCommandGroup(
                Commands.print("Run Rollers"),
                container.getCatzShooterFeeder().commandToShooter()
            ).until(()->container.getCatzShooterFeeder()
                                    .isNoteInShooterPosition()) // Until Shooter finalizes note position
        );
    }

    /** 
     * Runs a Simple auto aim to the speaker that can be overriden by driver
     */
    public static Command AutoAimShootNote(RobotContainer container, Supplier<Boolean> driverOveride) {
        return new SequentialCommandGroup(
            TransferNoteToShooter(container).unless(()->true),// Note is already in shooter
            new ParallelCommandGroup(
                new AUTO_AIM(),
                Commands.print("StartUp flywheels"),
                new SequentialCommandGroup(
                    Commands.waitUntil(()->(true && true)).unless(()->driverOveride.get()), // Until flywheels and shootersuperstructure are in position or driveroverride
                    container.getCatzShooterFeeder().commandShootNote()
                )
            )
        );
    }

    public static Command testSequence(RobotContainer container) {
        return Commands.sequence(container.getCatzShooterFeeder().commandShootNote(),
                                 Commands.waitSeconds(3),
                                 container.getCatzShooterFeeder().commandShootNote());
    }

    
}
