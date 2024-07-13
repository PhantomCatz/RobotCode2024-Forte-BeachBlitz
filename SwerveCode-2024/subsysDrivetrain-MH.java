package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
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
import frc.robot.util.EqualsUtil;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.swerve.SwerveSetpoint;


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

    // Variable Declaration
    private SwerveModuleState[] m_desiredStates = new SwerveModuleState[] {
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState(),
                                                        new SwerveModuleState()
                                                    };
                                                            
    private SwerveModuleState[] m_optimizedDesiredStates = new SwerveModuleState[] {
                                                                new SwerveModuleState(),
                                                                new SwerveModuleState(),
                                                                new SwerveModuleState(),
                                                                new SwerveModuleState()
                                                            };


    //----------------------------------------------------------------------------------------------
    //
    //  CatzDrivetrain() - Constructor
    //
    //----------------------------------------------------------------------------------------------
    // Private constructor for the singleton instance   //TBD says private but decl is public
    public CatzDrivetrain() {

        // Gyro Instantiation
        switch (CatzConstants.hardwareMode) {
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
        m_swerveModules[0] = new CatzSwerveModule(DriveConstants.moduleConfigs[0], 0); // TODO Map index numebrs to constants
        m_swerveModules[1] = new CatzSwerveModule(DriveConstants.moduleConfigs[1], 1);
        m_swerveModules[2] = new CatzSwerveModule(DriveConstants.moduleConfigs[2], 2);
        m_swerveModules[3] = new CatzSwerveModule(DriveConstants.moduleConfigs[3], 3);


        // Configure logging trajectories to advantage kit
        Pathfinding.setPathfinder(new LocalADStarAK());
        
        // PathPlanner Debug
        PathPlannerLogging.setLogActivePathCallback(
            (activepath)->{
                Logger.recordOutput("Obometry/Trajectory", activepath.toArray(new Pose2d[activepath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose)-> {
                Logger.recordOutput("Obometry/TrajectorySetpoint", targetPose);
            });

        gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.WARNING);  
    }

    //----------------------------------------------------------------------------------------------
    //
    //  periodic()                                //TBD - When is this called?
    //
    //----------------------------------------------------------------------------------------------
    @Override
    public void periodic() 
    {
        // Update inputs (sensors/encoders) for code logic and advantage kit
        for (CatzSwerveModule module : m_swerveModules) { 
            module.periodic();
        }

        //------------------------------------------------------------------------------------------
        //  Gyro TBD
        //------------------------------------------------------------------------------------------
        // Set Gyro Disconnect alert
        if (Robot.isReal()) {                                       //TBD isReal vs hardwareMode
            gyroDisconnected.set(!gyroInputs.gyroConnected);        //TBD where is gyroConnected set?
        }

        // Attempt to update gyro inputs and log
        try {
            gyroIO.updateInputs(gyroInputs);
        } catch (Exception e) {

        }
        Logger.processInputs("Drive/gyroinputs ", gyroInputs);    


        //------------------------------------------------------------------------------------------
        //  Swerve drive Odometry
        //------------------------------------------------------------------------------------------
        SwerveDriveWheelPositions wheelPositions = new SwerveDriveWheelPositions(getModulePositions());

        Rotation2d gyroAngle2d;

        // Grab latest gyro measurments
        if(CatzConstants.hardwareMode == CatzConstants.RobotHardwareMode.SIM) {     //TBD - This block should be a method in gyro class
            gyroAngle2d = null; 
        } else {
            gyroAngle2d = getDriveRotation2d();
        }

        CatzRobotTracker.getInstance().addOdometryObservation(new OdometryObservation(Timer.getFPGATimestamp(), 
                                                                                      wheelPositions, 
                                                                                      gyroAngle2d, 
                                                                                      getModuleStates(), 
                                                                                      gyroInputs.gyroAngleVel, 
                                                                                      gyroInputs.gyroAccelX, 
                                                                                      gyroInputs.gyroAccelY)   );

        //------------------------------------------------------------------------------------------
        //  Update current velocities use gyro when possible
        //------------------------------------------------------------------------------------------
        Twist2d robotRelativeVelocity = getTwist2dSpeeds();

        if (gyroInputs.gyroConnected == true) {
            robotRelativeVelocity.dtheta = Math.toRadians(gyroInputs.gyroAngleVel);     //TBD - is this the same gyroAngleVel used above? Looks like YES
        } else {
            robotRelativeVelocity.dtheta = robotRelativeVelocity.dtheta;                //TBD - do we need to do this?
        }


        robotRelativeVelocity.dtheta =
            gyroInputs.gyroConnected
                ? Math.toRadians(gyroInputs.gyroAngleVel)
                : robotRelativeVelocity.dtheta;

        CatzRobotTracker.getInstance().addVelocityData(robotRelativeVelocity);
   

        //------------------------------------------------------------------------------------------
        //  Logging
        //------------------------------------------------------------------------------------------
        SmartDashboard.putNumber("Heading", getGyroHeading());
    
        Logger.recordOutput("Drive/Pose",                       CatzRobotTracker.getInstance().getEstimatedPose());
        Logger.recordOutput("Drive/unoptimized module states",  m_desiredStates);
        Logger.recordOutput("Drive/optimized module states",    m_optimizedDesiredStates);

    }   //end of drivetrain periodic



    //----------------------------------------------------------------------------------------------
    //
    //  drive()
    //
    //----------------------------------------------------------------------------------------------
    public void drive(ChassisSpeeds chassisSpeeds, boolean useKinematics) 
    {

        //------------------------------------------------------------------------------------------
        //  Convert chassis speeds to individual module values and update module states.  If the
        //  useKinematics flag is TRUE, then update chassis speeds using discretize() method to  
        //  correct dynamics with "2nd order kinematics"
        //------------------------------------------------------------------------------------------
        if (useKinematics == true) 
        {
            SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, 
                                                                                                                                  0.02);    //TBD descrete sp, What was 0.02 again?
        } else {
            SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        }

        //------------------------------------------------------------------------------------------
        //  Scale down wheel speeds if > Max Velocity & update module states for each swerve module
        //------------------------------------------------------------------------------------------
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.driveConfig.maxLinearVelocity());

        //------------------------------------------------------------------------------------------
        //  Optimize wheel angles & update module states for each swerve module
        //------------------------------------------------------------------------------------------
        SwerveModuleState optimizedModuleState = new SwerveModuleState;       //TBD optimizedModuleState vs optimizedDesiredState since we no longer have desiredStates 

        for (int i = 0; i < 4; i++)     //TBD - Make 4 a constant
        {
            // The module returns the optimized state, useful for logging   TBD - what is this comment saying?
            optimizedModuleState = m_swerveModules[i].optimizeWheelAngles(moduleStates[i]);

            m_swerveModules[i].setModuleAngleAndVelocity(optimizedModuleState);
        }

    }   //End of drive()


    //--------------------------------------------------------------------------------------------------------------------------
    //          Drivetrain Driving methods
    //--------------------------------------------------------------------------------------------------------------------------
    /** chassis speeds input w/ correction for drift */
    public void driveWithDiscretizeKinematics(ChassisSpeeds chassisSpeeds) {
        // Correct dynamics with wpilib internal "2nd order kinematics" //TODO add boolean flag and consolidate methods
        ChassisSpeeds descreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        // Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(descreteSpeeds);
        setModuleStates(moduleStates);
    }

    /** chassis speeds input w/o any correction for drift */
    private void drive(ChassisSpeeds chassisSpeeds) { //TODO is characteraization run w/o descritization?
        // Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    /** Set individual module states to each of the swerve modules */
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        // Scale down wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.driveConfig.maxLinearVelocity());
        
        // Optimize wheel angles
        SwerveModuleState[] optimizedDesiredStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {  
            // The module returns the optimized state, useful for logging
            optimizedDesiredStates[i] = m_swerveModules[i].optimizeWheelAngles(desiredStates[i]);
        }

        // Set module states to each of the swerve modules
        LT_FRNT_MODULE.setModuleAngleAndVelocity(optimizedDesiredStates[0]);
        LT_BACK_MODULE.setModuleAngleAndVelocity(optimizedDesiredStates[1]);
        RT_BACK_MODULE.setModuleAngleAndVelocity(optimizedDesiredStates[2]);
        RT_FRNT_MODULE.setModuleAngleAndVelocity(optimizedDesiredStates[3]);
    }



    //-----------------------------------------------------------------------------------------------------------
    //      Drivetrain Misc Methods
    //-----------------------------------------------------------------------------------------------------------
    /** Runs forwards at the commanded voltage or amps. */
    public void runCharacterization(double input) {
        drive(new ChassisSpeeds(0.0, 0.0, input));
    }

    /** Get the position of all drive wheels in radians. */
    public double[] getWheelRadiusCharacterizationPosition() {
        return Arrays.stream(m_swerveModules).mapToDouble(CatzSwerveModule::getPositionRads).toArray();
    }

    /** Returns the average drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (var module : m_swerveModules) {
            driveVelocityAverage += module.getCharacterizationVelocityRadPerSec();
        }
        return driveVelocityAverage / 4.0;
    }

    /** Runs in a circle at omega. */
    public void runWheelRadiusCharacterization(double omegaSpeed) {
        drive(new ChassisSpeeds(0.0, 0.0, omegaSpeed));
    }

    /** Disables the characterization mode. */
    public void endCharacterization() {
        stopDriving();
    }

    /**
     * Returns command that orients all modules to {@code orientation}, ending when the modules have
     * rotated.
     */
    public Command orientModules(Rotation2d orientation) {
        return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
    }

    /**
     * Returns command that orients all modules to {@code orientations[]}, ending when the modules
     * have rotated.
     */
    public Command orientModules(Rotation2d[] orientations) {
        return run(() -> {
            for (int i = 0; i < orientations.length; i++) {
                m_swerveModules[i].setModuleAngleAndVelocity(
                    new SwerveModuleState(0.0, orientations[i]));
                    //new SwerveModuleState(0.0, new Rotation2d()));
            }
            })
            .until(
                () ->
                    Arrays.stream(m_swerveModules)
                        .allMatch(
                            module ->
                                EqualsUtil.epsilonEquals(
                                    module.getAngle().getDegrees(),
                                    module.getModuleState().angle.getDegrees(),
                                    2.0)))
            .withName("Orient Modules");
    }
    
    /**  Set brake mode for all swerve modules */
    public void setBrakeMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setBreakMode(true);
        }
    }

    /**  Set coast mode for all swerve modules */
    public void setCoastMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setBreakMode(false);
        }
    }

    /**  Create a command to stop driving */
    public void stopDriving() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.stopDriving();
            module.setSteerPower(0.0);
        }
    }

    /** command to cancel running auto trajectories */
    public Command cancelTrajectory() {
        return new InstantCommand();
    }

    public void resetDriveEncs() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }

    //-----------------------------------------------------------------------------------------------------------
    //      Drivetrain Getters
    //-----------------------------------------------------------------------------------------------------------
    /**
     * Dependant on the installation of the gyro, the value of this method may be negative
     * 
     * @return The Heading of the robot dependant on where it's been instantiated
     */
    private double getGyroHeading() {
        return -gyroInputs.gyroYawDegrees; //- for atlas //TODO need to verify on forte again
    }

    /** Get the Rotation2d object based on the gyro angle */
    private Rotation2d getDriveRotation2d() {
        return Rotation2d.fromDegrees(getGyroHeading());
    }

    /**  Get an array of swerve module states */
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            moduleStates[i] = m_swerveModules[i].getModuleState();
        }
        return moduleStates;
    }

    /** Returns the measured speeds of the robot in the robot's frame of reference. */
    @AutoLogOutput(key = "Drive/MeasuredSpeeds")
    private Twist2d getTwist2dSpeeds() {
        return DriveConstants.swerveDriveKinematics.toTwist2d(getModulePositions());
    }

    /**  Get an array of swerve module positions */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            modulePositions[i] = m_swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }

    /** Map Circle orientation for wheel radius characterization */
    public static Rotation2d[] getCircleOrientations() {
        return Arrays.stream(DriveConstants.moduleTranslations)
            .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
            .toArray(Rotation2d[]::new);
    }

}