package frc.robot.Autonomous;

import javax.swing.plaf.nimbus.NimbusLookAndFeel;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.Autonomous.commands.WaitUntilPassX;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.CatzSubsystems.Shooter.ShooterFlywheels.CatzShooterFlywheels;
import frc.robot.Commands.AutomatedSequenceCmds;
import frc.robot.Commands.CharacterizationCmds.FeedForwardCharacterization;
import frc.robot.Commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

public class CatzAutoRoutines {
    
    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");
    private RobotContainer m_container;

    private boolean trajectoriesLoaded = false;

    public CatzAutoRoutines(RobotContainer container) {
    
        this.m_container = container;
        //-------------------------------------------------------------------------------------------------------------------
        //   AUTON Priority LIST 
        //-------------------------------------------------------------------------------------------------------------------*/
        autoPathChooser.addOption("Test Auto", testAuto());
        autoPathChooser.addOption("StraightLine", testAuto2());
        autoPathChooser.addOption("Flywheel Characterization", flywheelCharacterization());


        System.out.println("registered");
        
    }

    private PathPlannerPath US_W1_3_1 = PathPlannerPath.fromPathFile("US_W1-3_1");
    private PathPlannerPath US_W1_3_2 = PathPlannerPath.fromPathFile("ver2 US_W1-3_2");
    private PathPlannerPath US_W1_3_3 = PathPlannerPath.fromPathFile("ver2 US_W1-3_3");
    private PathPlannerPath testPath  = PathPlannerPath.fromPathFile("Test");
    private PathPlannerPath straightLine = PathPlannerPath.fromPathFile("StraightLine"); 

    private Command testAuto() {
        preloadTrajectoryClass(US_W1_3_1);

        return new SequentialCommandGroup(
            new ParallelCommandGroup(new TrajectoryDriveCmd(testPath, m_container.getCatzDrivetrain()))
        );
    }

    private Command testAuto2(){
        preloadTrajectoryClass(straightLine);

        return new ParallelCommandGroup(
            new TrajectoryDriveCmd(straightLine, m_container.getCatzDrivetrain()),
            new WaitUntilPassX(2, new PrintCommand("heheheha"))
        );
    }

    //---------------------------------------------------------------------------------------------------------
    //          Characteration Routines
    //---------------------------------------------------------------------------------------------------------
    public Command flywheelCharacterization() {
        CatzShooterFlywheels flywheels = m_container.getCatzShooterFlywheels();
        return new FeedForwardCharacterization(flywheels, flywheels::runCharacterization, flywheels::getCharacterizationVelocity)
                        .withName("Flywheels characterization");
    }

    //---------------------------------------------------------------------------------------------------------
    //          Trajectory Helpers
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
