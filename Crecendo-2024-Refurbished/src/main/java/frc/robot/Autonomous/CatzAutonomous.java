package frc.robot.Autonomous;

import java.io.File;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
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

    private File pathsDirectory = new File(Filesystem.getDeployDirectory(), "choreo");
    private File autosDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");

    public CatzAutonomous(RobotContainer container) {
        this.m_container = container;

        CatzRobotTracker tracker = CatzRobotTracker.getInstance();
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(
            DriveConstants.driveConfig.maxLinearVelocity(), 
            DriveConstants.driveConfig.driveBaseRadius(),   
            new ReplanningConfig()
        );
        
        BooleanSupplier shouldFlip = ()->AllianceFlipUtil.shouldFlipToRed();
        AutoBuilder.configureHolonomic(
            tracker::getEstimatedPose,
            tracker::resetPosition,
            tracker::getRobotChassisSpeeds,
            container.getCatzDrivetrain()::drive,
            config,
            shouldFlip,
            container.getCatzDrivetrain()
        );

        NamedCommands.registerCommand("PrintCMD", Commands.print("HI"));

        for(File pathFile : pathsDirectory.listFiles()){
            String pathName = pathFile.getName().replaceFirst("[.][^.]+$", ""); //to get rid of the extensions trailing the path names
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            NamedCommands.registerCommand(pathName, new TrajectoryDriveCmd(path, container.getCatzDrivetrain()));
        }
        for (File autoFile: autosDirectory.listFiles()){
            String autoName = autoFile.getName().replaceFirst("[.][^.]+$", "");
            autoPathChooser.addOption(autoName, new PathPlannerAuto(autoName));
        }
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
