package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.Robot;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker.OdometryObservation;
import frc.robot.util.Alert;
import frc.robot.util.swerve.SwerveSetpoint;
//import frc.robot.util.swerve.SwerveSetpointGenerator;

// Drive train subsystem for swerve drive implementation
public class CatzDrivetrain extends SubsystemBase {

    // Gyro input/output interface
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Alerts
    private final Alert gyroDisconnected;

    // Array of swerve modules representing each wheel in the drive train
    private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];

    // Swerve modules representing each corner of the robot
    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;

    // boolean for determining whether to use vision estimates in pose estimation
    private boolean isVisionEnabled = true;

    private SwerveSetpoint currentSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
            });
  // private final SwerveSetpointGenerator setpointGenerator; //TODO add back in swervesetpointgenerator

    // Private constructor for the singleton instance
    public CatzDrivetrain() {
        
        // Determine gyro input/output based on the robot mode
        switch (CatzConstants.currentMode) {
            case REAL:
                gyroIO = new GyroIONavX();
                break;
            case REPLAY:
                gyroIO = new GyroIONavX() {};
                break;
            default:
                gyroIO = null;
                break;
        }

        // Create swerve modules for each corner of the robot
        LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.moduleConfigs[0], 0);
        LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.moduleConfigs[1], 1);
        RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.moduleConfigs[2], 2);
        RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.moduleConfigs[3], 3);

        // Assign swerve modules to the array for easier access
        m_swerveModules[0] = LT_FRNT_MODULE;
        m_swerveModules[1] = LT_BACK_MODULE;
        m_swerveModules[2] = RT_BACK_MODULE;
        m_swerveModules[3] = RT_FRNT_MODULE;

        // setpointGenerator =
        // SwerveSetpointGenerator.builder()
        //     .kinematics(DriveConstants.kinematics)
        //     .moduleLocations(DriveConstants.moduleTranslations)
        //     .build();
        //Configure logging trajectories to advantage kit
       // Pathfinding.setPathfinder(new LocalADStarAK());
        
        //DEBUG
        PathPlannerLogging.setLogActivePathCallback(
            (activepath)->{
                Logger.recordOutput("Obometry/Trajectory", activepath.toArray(new Pose2d[activepath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose)-> {
                Logger.recordOutput("Obometry/TrajectorySetpoint", targetPose);
            });

        gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.WARNING);

       // gyroIO.resetNavXIO(0);  //TBD if red alliance how does the gryo get reset
        
    }

    @Override
    public void periodic() {
        // Update inputs (sensors/encoders) for code logic and advantage kit
        for (CatzSwerveModule module : m_swerveModules) {
            module.periodic();
        }

        if (Robot.isReal()) {
            gyroDisconnected.set(!gyroInputs.gyroConnected);
        }

        //attempt to update gyro inputs and log
        try {
            gyroIO.updateInputs(gyroInputs);
        } catch (Exception e) {

        }
        Logger.processInputs("Drive/gyroinputs ", gyroInputs);    


        // Swerve drive Odometry
        SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(getModulePositions());
        Rotation2d gyroAngle2d;
        // Grab latest gyro measurments
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
            gyroAngle2d = null;
        } else {
            gyroAngle2d = getRotation2d();
        }
        CatzRobotTracker.getInstance().addOdometryObservation(new OdometryObservation(Logger.getRealTimestamp(), 
                                                                                      wheelPositions, 
                                                                                      gyroAngle2d, 
                                                                                      getModuleStates(), 
                                                                                      gyroInputs.gyroAngleVel, 
                                                                                      gyroInputs.gyroAccelX, 
                                                                                      gyroInputs.gyroAccelY));
   

        //------------------------------------------------------------------------------------------------
        // Logging
        //------------------------------------------------------------------------------------------------
       
        //DEBUG
        // Logger.recordOutput("Obometry/LimelightPose Soba" , vision.getVisionOdometry().get(1).getPose()); 
        // Logger.recordOutput("Obometry/LimelightPose Udon" , vision.getVisionOdometry().get(2).getPose()); 
        SmartDashboard.putNumber("gyroAngle", getGyroAngle());
        Logger.recordOutput("Pose", CatzRobotTracker.getInstance().getEstimatedPose());

    }   //end of drivetrain periodic

    public void driveWithDiscretizeKinematics(ChassisSpeeds chassisSpeeds) {

        //correct dynamics with wpilib internal "2nd order kinematics"
        ChassisSpeeds descreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        // Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(descreteSpeeds);
        setModuleStates(moduleStates);
    }

    private void drive(ChassisSpeeds chassisSpeeds) { //TODO is characteraization run w/o descritization?
        
        // Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    // Set individual module states to each of the swerve modules
    private void setModuleStates(SwerveModuleState[] desiredStates) {

        // Scale down wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.driveConfig.maxLinearVelocity());

        //optimize wheel angles
        SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {  
            // The module returns the optimized state, useful for logging
            optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(desiredStates[i]);
        }

        // Set module states to each of the swerve modules
        LT_FRNT_MODULE.setDesiredState(optimizedDesiredStates[0]);
        LT_BACK_MODULE.setDesiredState(optimizedDesiredStates[1]);
        RT_BACK_MODULE.setDesiredState(optimizedDesiredStates[2]);
        RT_FRNT_MODULE.setDesiredState(optimizedDesiredStates[3]);

       
    }

    //--------------------------------------------------DEBUG PURPOSES LOGS-------------------------------------------------
    public void debugLogsDriveSubSys(){
        // Logger.recordOutput("Drive/unoptimized module states", desiredStates);
        // Logger.recordOutput("Drive/optimized module states", optimizedDesiredStates);
    }

    //--------------------------------------------------DriveTrain MISC methods-------------------------------------------------

    /** Runs forwards at the commanded voltage or amps. */
    public void runCharacterization(double input) {

        drive(new ChassisSpeeds(0.0, 0.0, input));
    }

    /** Get the position of all drive wheels in radians. */
    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(m_swerveModules).mapToDouble(CatzSwerveModule::getPositionRads).toArray();
    }

    /** Runs in a circle at omega. */
    public void runWheelRadiusCharacterization(double omegaSpeed) {

        drive(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
    }

    /** Disables the characterization mode. */
    public void endCharacterization() {

    }
    
    // Set brake mode for all swerve modules
    public void setBrakeMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setBreakMode(true);
        }
    }

    // Set coast mode for all swerve modules
    public void setCoastMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setBreakMode(false);
        }
    }

    // Create a command to stop driving
    public void stopDriving() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.stopDriving();
            module.setSteerPower(0.0);
        }
    }

    //command to cancel running auto trajectories
    public Command cancelTrajectory() {
        return new InstantCommand();
    }
    //----------------------------------------------Gyro methods----------------------------------------------


    //TBD do not use unless autoaim does not work anymore
    public void flipGyro() {
        gyroIO.setAngleAdjustmentIO(180);
    }

    public Command resetGyro() {
        return runOnce(() -> {
            if(CatzConstants.choosenAllianceColor == AllianceColor.Red){
                gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw + 180);
            }else{
                gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
            }
        });
    }

    // Get the gyro angle (negative due to the weird coordinate system)
    public double getGyroAngle() {
        return -gyroInputs.gyroAngle; //- for atlas
    }

    // Get the roll angle of the gyro
    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }

    // Get the heading of the robot in a integer quantity
    public double getHeading() {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    // Get the heading of the robot in radians
    public double getHeadingRadians() {
        return (getHeading() * Math.PI / 180);
    }

    // Get the Rotation2d object based on the gyro angle
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    //----------------------------------------------Enc resets-------------------------------------------------------

    // Reset drive encoders for all swerve modules
    public void resetDriveEncs() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }

    // Get an array of swerve module states
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            moduleStates[i] = m_swerveModules[i].getModuleState();
        }
        return moduleStates;
    }

    // Reset the position of the robot with a given pose
    public void resetPosition(Pose2d pose) {
        double angle = pose.getRotation().getDegrees();

        //gyroIO.setAngleAdjustmentIO(angle);
        pose = new Pose2d(pose.getTranslation(), Rotation2d.fromDegrees(angle));
        CatzRobotTracker.getInstance().resetPoseEstimator(Rotation2d.fromDegrees(angle),getModulePositions(),pose);
    }

    // Get an array of swerve module positions
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            modulePositions[i] = m_swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }

    //----------------------------------------vision-----------------------------------------
    public Command toggleVisionEnableCommand() {
        if(isVisionEnabled == true) {
            return run(()-> setVisionEnable(false));
        }
        else {
            return run(()-> setVisionEnable(true));
        }
    }

    //access method for determining whether to use vision in pose estimation
    private void setVisionEnable(boolean enable) {
        isVisionEnabled = enable;
    }

}