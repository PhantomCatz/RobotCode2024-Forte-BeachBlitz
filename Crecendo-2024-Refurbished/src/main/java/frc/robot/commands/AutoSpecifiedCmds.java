package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;

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

    private PathPlannerPath testPath  = PathPlannerPath.fromPathFile("DriveStraightMid");

    private Command testAuto(RobotContainer container) {
        return new SequentialCommandGroup(
            setAutonStartPose(testPath)
        );
    }
    private Command setAutonStartPose(PathPlannerPath startPath){
        return Commands.runOnce(()->{
            PathPlannerPath path = startPath;
            if(CatzConstants.choosenAllianceColor == CatzConstants.AllianceColor.Red) {
                path = startPath.flipPath();
            }

            // m_container.getCatzDrivetrain().resetPosition(path.getPreviewStartingHolonomicPose());
        });
    }

    //configured dashboard
    public Command getCommand() { //TODO fix back to network tables
        return testAuto(m_container);
    }
}
