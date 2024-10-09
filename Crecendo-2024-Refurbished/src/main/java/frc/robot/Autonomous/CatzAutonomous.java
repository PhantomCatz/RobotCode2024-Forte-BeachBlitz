package frc.robot.Autonomous;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;
import frc.robot.CatzSubsystems.Shooter.ShooterFeeder.CatzShooterFeeder;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.CatzSubsystems.SuperSubsystem.CatzSuperSubsystem;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Commands.DriveAndRobotOrientationCmds.WaitUntilPassX;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.AllianceFlipUtil.PathPlannerFlippingState;

public class CatzAutonomous {
    
    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");
    private RobotContainer m_container;

    private boolean trajectoriesLoaded = false;
    private boolean isPathPlannerFlipped = false;

    private PathPlannerPath testPath;
    private PathPlannerPath UpperSpeakerGamepiece1;
    private PathPlannerPath UpperSpeakerGamepiece2; 
    private PathPlannerPath straightLine;


    public CatzAutonomous(RobotContainer container) {

        this.m_container = container;
        // Declare Paths
        testPath  = PathPlannerPath.fromPathFile("Test");
        UpperSpeakerGamepiece1 = PathPlannerPath.fromPathFile("UpperSpeakerGamepiece1");
        UpperSpeakerGamepiece2 = PathPlannerPath.fromPathFile("UpperSpeakerGamepiece2");
        straightLine = PathPlannerPath.fromPathFile("StraightLine");

        //   AUTON Priority LIST 
        autoPathChooser.addOption("Test Auto", testAuto());
        autoPathChooser.addOption("UpperSpeakerWing1center1", upperSpeakerWing1Center1());



        autoPathChooser.addOption("Flywheel Characterization", flywheelCharacterization());
        autoPathChooser.addOption("StraightLine", straightLine());

        NamedCommands.registerCommand("PrintCMD", Commands.print("HI")); // TODO these comands are broken
        
    }

    private Command upperSpeakerWing1Center1() {
        CatzDrivetrain drivetrain = m_container.getCatzDrivetrain();
        CatzSuperSubsystem superSubsystem = m_container.getCatzSuperstructure();
        CatzShooterFeeder feeder = m_container.getCatzShooterFeeder();

        List<Double> waypoints = Arrays.asList(0.1,0.2);

        List<Command> commandSequenceOne = Arrays.asList(AutomatedSequenceCmds.scoreSpeakerAutoAim(m_container, ()->false));

        return new SequentialCommandGroup(
                new TrajectoryDriveCmd(UpperSpeakerGamepiece1, drivetrain, waypoints, commandSequenceOne)
                    .until(()->true)//feeder.isNoteInRestingPosition())
                    .andThen(Commands.print("Intaking")).alongWith(Commands.print("Scoring")),
            new ParallelCommandGroup(
                //new TrajectoryDriveCmd(UpperSpeakerGamepiece2, drivetrain),
                AutomatedSequenceCmds.scoreSpeakerAutoAim(m_container, ()-> false).alongWith(Commands.print("shooting"))
            )
        );
    }


    private Command testAuto() {

        preloadTrajectoryClass(testPath);
        List<Double> waypoints = Arrays.asList(0.3,0.8);

        List<Command> commandSequenceOne = Arrays.asList(Commands.print("HI"), Commands.print("2+2"));


        return new SequentialCommandGroup(
            new ParallelCommandGroup(new TrajectoryDriveCmd(testPath, m_container.getCatzDrivetrain(), waypoints, commandSequenceOne))
        );
    }

    private Command straightLine(){
        preloadTrajectoryClass(straightLine);

        return new SequentialCommandGroup(
        );
    }

    //---------------------------------------------------------------------------------------------------------
    //
    //          Characteration Routines
    //
    //---------------------------------------------------------------------------------------------------------
    public Command flywheelCharacterization() {
        CatzShooterFlywheels flywheels = m_container.getCatzShooterFlywheels();
        return new FeedForwardCharacterization(flywheels, flywheels::runCharacterization, flywheels::getCharacterizationVelocity)
                        .withName("Flywheels characterization");
    }
    

    //Automatic pathfinding command
    public Command autoFindPathSpeakerLOT() {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(180)),
                new Pose2d(1.50, 0.69, Rotation2d.fromDegrees(235))
                    );

        //send path info to trajectory following command
        return new TrajectoryDriveCmd(bezierPoints, 
                                      DriveConstants.autoPathfindingConstraints, 
                                      new GoalEndState(0.0, Rotation2d.fromDegrees(235)), m_container.getCatzDrivetrain());
    }

    //---------------------------------------------------------------------------------------------------------
    //
    //          Trajectory Helpers
    //
    //---------------------------------------------------------------------------------------------------------
    private void preloadTrajectoryClass(PathPlannerPath segment) {
        // This is done because Java loads classes lazily. Calling this here loads the trajectory pathplanner class which
        // is used to follow paths and saves user code ms loop time at the start of auto.
        if (!trajectoriesLoaded) {
            trajectoriesLoaded = true;
            var trajectory = new PathPlannerTrajectory(
                    segment,
                    CatzRobotTracker.getInstance().getRobotChassisSpeeds(),
                    CatzRobotTracker.getInstance().getEstimatedPose().getRotation());
        }
    }

    /** Getter for final autonomous routine */
    public Command getCommand() { 
        return autoPathChooser.get();
    }
}
