package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.Shooter.ShooterFeeder.CatzShooterFeeder;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.CatzSubsystems.IntakeRollers.CatzIntakeRollers;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem.SuperstructureState;

/** Place where any sequencing/nonPosition Based robot state logic is held */
public class AutomatedSequenceCmds {
    /** 
     * Runs the Auto Note detect in telop to feed note into shooter
     */
    public static Command noteDetectIntakeToShooter(RobotContainer container) {
        // declare subsystem requirements
        CatzIntakeRollers rollers = container.getCatzIntakeRollers();
        CatzSuperSubsystem superstructure = container.getCatzSuperstructure();

        // return command sequence
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                superstructure.setSuperStructureState(SuperstructureState.INTAKE_GROUND), // Until Intake has made it to final ground pos
                rollers.setRollersIn()
            ).until(() -> rollers.getBeamBreak()), // Until Intake Rollers have detected note,
            transferNoteToShooter(container) //Stow is already called in method
        );
    }

    /** 
     * Runs the Auto Note detect in telop to feed note into Intake for amp Scoring
     */
    public static Command noteDetectIntakeToAmpScoring(RobotContainer container) {
        CatzSuperSubsystem superstructure = container.getCatzSuperstructure();
        CatzIntakeRollers rollers = container.getCatzIntakeRollers();

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                superstructure.setSuperStructureState(SuperstructureState.INTAKE_GROUND), // Until Intake has made it to final ground pos
                rollers.setRollersIn()
            ).until(()-> rollers.getBeamBreak()), // Until Intake Rollers have detected note,
            superstructure.setSuperStructureState(SuperstructureState.STOW) // Until Intake has stowed 
        );
    }

    /** 
     * Runs shooter to intake note transfer
     */
    public static Command transferNoteToIntake(RobotContainer container) {
        CatzSuperSubsystem superstructure = container.getCatzSuperstructure();
        CatzIntakeRollers rollers = container.getCatzIntakeRollers();
        CatzShooterFeeder feeder = container.getCatzShooterFeeder();


        return new SequentialCommandGroup(
            superstructure.setSuperStructureState(SuperstructureState.STOW).until(()->superstructure.isIntakeInPosition()),
            new ParallelCommandGroup(
                rollers.setRollersHandofftoIntake(),
                feeder.commandToIntake()
            ).until(()->rollers.getBeamBreak()) // Until intake finalizes note position  
        );
    }

    /** 
     * Runs intake to shooter note transfer
     */
    public static Command transferNoteToShooter(RobotContainer container) {
        CatzSuperSubsystem superstructure = container.getCatzSuperstructure();
        CatzIntakeRollers rollers = container.getCatzIntakeRollers();
        CatzShooterFeeder feeder = container.getCatzShooterFeeder();

        return new SequentialCommandGroup(
            superstructure.setSuperStructureState(SuperstructureState.STOW).until(()->superstructure.isIntakeInPosition()), // Until Intake has stowed 
            Commands.waitSeconds(0.2),
            new ParallelCommandGroup(
                rollers.setRollersHandofftoShooter(),
                feeder.commandToShooter()
            ).until(()->container.getCatzShooterFeeder()
                                    .isNoteInShooterPosition()) // Until Shooter finalizes note position
        );
    }

    /** 
     * Runs a Simple auto aim to the speaker that can be overriden by driver
     */
    public static Command AutoAimShootNote(RobotContainer container, Supplier<Boolean> driverOveride) {
        CatzSuperSubsystem superstructure = container.getCatzSuperstructure();
        CatzShooterFlywheels flywheels = container.getCatzShooterFlywheels();
        CatzShooterFeeder feeder = container.getCatzShooterFeeder();

        return new SequentialCommandGroup(
            transferNoteToShooter(container).unless(()->feeder.isNoteInShooterPosition()),// Note is already in shooter
            new ParallelCommandGroup(
                superstructure.setSuperStructureState(SuperstructureState.AUTO_AIM),
                //flywheels.shootCommand(),
                new SequentialCommandGroup(
                    Commands.waitUntil(()->(flywheels.atGoal() && true)).unless(()->driverOveride.get()), // Until flywheels and shootersuperstructure are in position or driveroverride
                    container.getCatzShooterFeeder().commandShootNote()
                )
            )
        );
    }

    public static Command testSequence(RobotContainer container) {
        return Commands.sequence(container.getCatzShooterFeeder().commandShootNote(),
                                 Commands.waitSeconds(3),
                                 Commands.print("It worked"),
                                 container.getCatzShooterFeeder().commandShootNote());
    }

    // public static Command ShooterToScoreAmp(RobotContainer container) {
    //     CatzSuperSubsystem superstructure = container.getCatzSuperstructure();

    //     return new SequentialCommandGroup(
    //         transferNoteToIntake(container),
    //         superstructure.setSuperStructureState(SuperstructureState.SCORE_AMP).until(()->superstructure.isIntakeInPosition())
    //     );
    // }
}
