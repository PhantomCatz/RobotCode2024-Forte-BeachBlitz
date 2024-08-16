package frc.robot.Commands.DriveAndRobotOrientationCmds;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;

import java.util.List;

public class TrajectoryDriveCmd extends Command {

    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;

    private final HolonomicDriveController hocontroller;
    private CatzDrivetrain m_driveTrain;
    private PathPlannerTrajectory trajectory;
    
    private final Timer timer = new Timer();
    private final double TIMEOUT_RATIO = 5;
    private PathPlannerPath path;

    /**
     * @param drivetrain           The coordinator between the gyro and the swerve modules.
     * @param trajectory          The trajectory to follow.
     */
    public TrajectoryDriveCmd(PathPlannerPath newPath, CatzDrivetrain drivetrain) {
        path = newPath;
        m_driveTrain = drivetrain;

        hocontroller = DriveConstants.holonomicDriveController;
        addRequirements(m_driveTrain);
    }

    //Auto Pathplanning trajectoreies
    public TrajectoryDriveCmd(List<Translation2d> bezierPoints, 
                              PathConstraints constraints, 
                              GoalEndState endRobotState,
                              CatzDrivetrain drivetrain) {
        PathPlannerPath newPath = new PathPlannerPath(bezierPoints, constraints, endRobotState);
        path = newPath;
        m_driveTrain = drivetrain;

        hocontroller = DriveConstants.holonomicDriveController;

        addRequirements(m_driveTrain);
    }

    private boolean atTarget = false;
    private double pathTimeOut;

    @Override
    public void initialize() {
        // Reset and begin timer
        timer.reset();
        timer.start();


        // Flip auton path to mirrored red side if we choose red alliance 
        if(CatzConstants.choosenAllianceColor == CatzConstants.AllianceColor.Red) {
            path = path.flipPath();
        }

        // Create pathplanner trajectory
        this.trajectory = new PathPlannerTrajectory(
                                path, 
                                DriveConstants.
                                    swerveDriveKinematics.
                                        toChassisSpeeds(CatzRobotTracker.getInstance().getRobotSwerveModuleStates()),
                                CatzRobotTracker.getInstance().getRobotRotation());
                                
        pathTimeOut = trajectory.getTotalTimeSeconds() * TIMEOUT_RATIO;

    }

    @Override
    public void execute() {
        if(!atTarget){

            double currentTime = this.timer.get();
    
            // Getters from pathplanner and current robot pose
            PathPlannerTrajectory.State goal = trajectory.sample(currentTime);
            Rotation2d targetOrientation     = goal.targetHolonomicRotation;
            Pose2d currentPose               = CatzRobotTracker.getInstance().getEstimatedPose();
                
            /* 
            * Convert PP trajectory into a wpilib trajectory type 
            * Only takes in the current robot position 
            * Does not take acceleration to be used with the internal WPILIB trajectory library
            */
    
            Trajectory.State state = new Trajectory.State(currentTime, 
                                                          0.0,  //made the holonomic drive controller only rely on its current position, not its velocity because the target velocity is used as a ff
                                                          0.0, 
                                                          new Pose2d(goal.positionMeters, new Rotation2d()),/*new Pose2d(currentPose.getTranslation().plus(displacement), new Rotation2d()*/
                                                          0.0);
    
            //construct chassisspeeds
            ChassisSpeeds adjustedSpeeds = hocontroller.calculate(currentPose, state, targetOrientation);

            //send to drivetrain
            m_driveTrain.drive(adjustedSpeeds, true);

        }else{
            m_driveTrain.stopDriving();
        }

    }

    /*
     * For Debugging Purposes 
     * Keep them commmented ALWAYS if you are not using it 
     */
    public void debugLogsTrajectory(){
        //Logger.recordOutput("Desired Auto Pose", new Pose2d(state.poseMeters.getTranslation(), goal.targetHolonomicRotation));
        //Logger.recordOutput("Adjusted Speeds X", adjustedSpeeds.vxMetersPerSecond);
        //Logger.recordOutput("Adjusted Speeds Y", adjustedSpeeds.vyMetersPerSecond);
        //Logger.recordOutput("Trajectory Goal MPS", state.velocityMetersPerSecond);
        //Logger.recordOutput("PathPlanner Goal MPS", goal.velocityMps);

        //System.out.println(goal.getTargetHolonomicPose());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop(); // Stop timer
        m_driveTrain.stopDriving();
        System.out.println("trajectory done");
    }


    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        double currentPosX =        CatzRobotTracker.getInstance().getEstimatedPose().getX();
        double currentPosY =        CatzRobotTracker.getInstance().getEstimatedPose().getY();
        double currentRotation =    CatzRobotTracker.getInstance().getEstimatedPose().getRotation().getDegrees();

        double desiredPosX =        trajectory.getEndState().positionMeters.getX();
        double desiredPosY =        trajectory.getEndState().positionMeters.getY();
        double desiredRotation =    trajectory.getEndState().targetHolonomicRotation.getDegrees();

        double xError =        Math.abs(desiredPosX - currentPosX);
        double yError =        Math.abs(desiredPosY - currentPosY);
        double rotationError = Math.abs(desiredRotation - currentRotation);

        atTarget = (xError < ALLOWABLE_POSE_ERROR && 
                    yError < ALLOWABLE_POSE_ERROR && 
                    rotationError < ALLOWABLE_ROTATION_ERROR);

        return atTarget || timer.hasElapsed(pathTimeOut);

    } 

}