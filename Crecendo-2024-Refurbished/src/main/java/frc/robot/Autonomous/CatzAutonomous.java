package frc.robot.Autonomous;

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
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;
import frc.robot.Utilities.AllianceFlipUtil;
import frc.robot.Utilities.AllianceFlipUtil.PathPlannerFlippingState;

public class CatzAutonomous {
    
    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");
    private RobotContainer m_container;

    private boolean trajectoriesLoaded = false;
    private boolean isPathPlannerFlipped = false;

    private PathPlannerPath US_W1_3_1; 
    private PathPlannerPath US_W1_3_2; 
    private PathPlannerPath US_W1_3_3; 
    private PathPlannerPath testPath ; 


    public CatzAutonomous(RobotContainer container) {

        this.m_container = container;
        // Declare Paths
        US_W1_3_1 = PathPlannerPath.fromPathFile("US_W1-3_1");
        US_W1_3_2 = PathPlannerPath.fromPathFile("ver2 US_W1-3_2");
        US_W1_3_3 = PathPlannerPath.fromPathFile("ver2 US_W1-3_3");
        testPath  = PathPlannerPath.fromPathFile("Test");

        //   AUTON Priority LIST 
        autoPathChooser.addOption("Test Auto", testAuto());
        autoPathChooser.addOption("Flywheel Characterization", flywheelCharacterization());

        NamedCommands.registerCommand("PrintCMD", Commands.print("HI")); // TODO these comands are broken
        
    }


    private Command testAuto() {

        preloadTrajectoryClass(US_W1_3_1);
        

        return new SequentialCommandGroup(
            new ParallelCommandGroup(new TrajectoryDriveCmd(testPath, m_container.getCatzDrivetrain()))
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
        return testAuto();
    }
}
