package frc.robot.Utilities;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;

public class AutoAimingParametersUtil {

    private static AimingParameters latestParameters;
    private static final LoggedTunableNumber autoLookahead = new LoggedTunableNumber("RobotState/AutoLookahead", 0.5);
    private static final LoggedTunableNumber lookahead = new LoggedTunableNumber("RobotState/lookaheadS", 0.35);
    private static final LoggedTunableNumber superPoopLookahead = new LoggedTunableNumber("RobotState/SuperPoopLookahead", 0.1);
    private static final LoggedTunableNumber closeShootingZoneFeet = new LoggedTunableNumber("RobotState/CloseShootingZoneFeet", 10.0);


    /**
     * 
     *  Credit: Jetstream 2710
     * 
     */
    public static AimingParameters getAutoAimSpeakerParemeters() {
        if (latestParameters != null) {
            // Cache previously calculated aiming parameters. Cache is invalidated whenever new
            // observations are added.
            return latestParameters;
        
        }
        // Collect Given Variables
        double vx = CatzRobotTracker.getInstance().getRobotChassisSpeeds().vxMetersPerSecond;
        double vy = CatzRobotTracker.getInstance().getRobotChassisSpeeds().vyMetersPerSecond;
        Translation2d robotPose = new Translation2d(CatzRobotTracker.getInstance().getEstimatedPose().getX(), 
                                                    CatzRobotTracker.getInstance().getEstimatedPose().getY());

        // Contstruct translation objects for vector addition
        Translation3d targetPose3d = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening);
        Translation2d targetPose = new Translation2d(targetPose3d.getX(), targetPose3d.getY()); 
        Logger.recordOutput("AutoAim/targetpose3d", targetPose3d);
        Logger.recordOutput("AutoAim/targetPose", targetPose);
    


        // Target Robot Shooting Horizontal Angle
        Vector<N2> addedVelocity = VecBuilder.fill(vx , vy); 
        Vector<N2> robotToTarget = VecBuilder.fill(targetPose.getX() - robotPose.getX()  , targetPose.getY() - robotPose.getY());
        //Vector<N2> scaledRobotToTarget = robotToTarget.times(shooter_velocity/robotToTarget.norm());
        Vector<N2> correctVector = robotToTarget;//.minus(addedVelocity); // Account for robot velocity by subtracting vectors TODO fix later
        double feildRelTargetRad = Math.atan(correctVector.get(1,0)/correctVector.get(0,0)); // Take arctangent to find feild relative target rotation
        
        Logger.recordOutput("AutoAim/targetAngle", Math.toDegrees(feildRelTargetRad));
       
       
       // Conversion to Turret Angle
        double targetTurretDegree = Math.toDegrees(feildRelTargetRad);    //Convert from radians to deg
        if(targetTurretDegree > 180) {         // Roll back if angle is past the softlimit
          targetTurretDegree = targetTurretDegree - 360;
        } else if(targetTurretDegree < -180) {
          targetTurretDegree = targetTurretDegree + 360;
        }
        Logger.recordOutput("AutoAim/added velocity Vector", addedVelocity.norm());
        Logger.recordOutput("AutoAim/robotToTarget Vector", robotToTarget.norm());
        Logger.recordOutput("AutoAim/Final Vector", correctVector.norm());
        Logger.recordOutput("AutoAim/After Rollback targetAngle", targetTurretDegree);



        // Target Robot Shooter Elevation Angle
        double targetDistance = targetPose.getDistance(robotPose);
        double elevationAngle = shooterPivotTable.get(targetDistance);

        Logger.recordOutput("AutoAim/targetDistance", targetDistance);
        Logger.recordOutput("AutoAim/TargetElevationAngle", elevationAngle);

        latestParameters =
            new AimingParameters(
                Rotation2d.fromDegrees(targetTurretDegree),
                Rotation2d.fromDegrees(elevationAngle),
                targetDistance,
                new FlywheelSpeeds(0, 0));
        return latestParameters;
    }


    public record AimingParameters(
        Rotation2d turretHeading,
        Rotation2d shooterPivotAngle,
        double effectiveDistance,
        FlywheelSpeeds flywheelSpeeds) {}

    public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {
        public static FlywheelSpeeds interpolate(FlywheelSpeeds t1, FlywheelSpeeds t2, double v) {
        double leftSpeed = MathUtil.interpolate(t1.leftSpeed(), t2.leftSpeed(), v);
        double rightSpeed = MathUtil.interpolate(t1.rightSpeed(), t2.rightSpeed(), v);
        return new FlywheelSpeeds(leftSpeed, rightSpeed);
        }
    }

    //------------------------------------------------------------------------------------------------
    //  Shooter EL angle look up table key: 
    //    Param 1: Distance in meters from back wall to Center of the robot
    //    Param 2: pivot position % of max elevation units
    // TBD - how did we determine distance interval?
    // TBD - explain why two distance values
    //------------------------------------------------------------------------------------------------
    private static final InterpolatingDoubleTreeMap shooterPivotTable = new InterpolatingDoubleTreeMap();

    static {
        shooterPivotTable.put(1.478, 1.0);

        shooterPivotTable.put(1.875, 0.885);
        // newShooterPivotTable.put(1.875, 0.82);
        // newShooterPivotTable.put(1.875, 0.95);

        shooterPivotTable.put(2.875, 0.485);
        // newShooterPivotTable.put(2.875, 0.42);
        // newShooterPivotTable.put(2.875, 0.55);
        
        shooterPivotTable.put(3.875, 0.26);
        // newShooterPivotTable.put(3.875, 0.21);
        // newShooterPivotTable.put(3.875, 0.31);
        
        shooterPivotTable.put(4.875, 0.095);
        // newShooterPivotTable.put(4.875, 0.09);
        // newShooterPivotTable.put(4.875, 0.1);

        shooterPivotTable.put(5.875, 0.02);
        // newShooterPivotTable.put(5.875, 0.0);
        // newShooterPivotTable.put(5.875, 0.04);

        shooterPivotTable.put(6.813, 0.0);
    }

}
