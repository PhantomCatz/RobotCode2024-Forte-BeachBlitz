package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveAndRobotOrientationCmds.TrajectoryDriveCmd;

public class AutoSpecifiedCmds {
    
    private static LoggedDashboardChooser<Command> autoPathChooser = new LoggedDashboardChooser<>("Chosen Autonomous Path");
    private RobotContainer m_container;

    public AutoSpecifiedCmds(RobotContainer container) {
    
        this.m_container = container;
        /*-------------------------------------------------------------------------------------------------------------------
        *
        *   AUTON Priority LIST (It's in order - So Don't Mess it Up)
        *
        *-------------------------------------------------------------------------------------------------------------------*/
    
        autoPathChooser.addOption("Test Auto", testAuto(container));

    }

    private PathPlannerPath US_W1_3_1 = PathPlannerPath.fromPathFile("US_W1-3_1");
    private PathPlannerPath US_W1_3_2 = PathPlannerPath.fromPathFile("ver2 US_W1-3_2");
    private PathPlannerPath US_W1_3_3 = PathPlannerPath.fromPathFile("ver2 US_W1-3_3");
    private PathPlannerPath testPath  = PathPlannerPath.fromPathFile("DriveStraightMid");

    private Command testAuto(RobotContainer container) {
        return new SequentialCommandGroup(
            setAutonStartPose(testPath),

            new ParallelCommandGroup(new TrajectoryDriveCmd(testPath, container.getCatzDrivetrain()))
        );
    }
    private Command setAutonStartPose(PathPlannerPath startPath){
        return Commands.runOnce(()->{
            PathPlannerPath path = startPath;
            if(CatzConstants.choosenAllianceColor == CatzConstants.AllianceColor.Red) {
                path = startPath.flipPath();
            }

            m_container.getCatzDrivetrain().resetPosition(path.getPreviewStartingHolonomicPose());
        });
    }

    //configured dashboard
    public Command getCommand() { //TODO fix back to network tables
        return testAuto(m_container);
    }
}
