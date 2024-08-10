package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.Subsystems.DriveAndRobotOrientation.CatzRobotTracker;

public class AutoAimingParametersUtil {

    private AimingParameters latestParameters;
    private static final LoggedTunableNumber autoLookahead =
        new LoggedTunableNumber("RobotState/AutoLookahead", 0.5);
    private static final LoggedTunableNumber lookahead =
        new LoggedTunableNumber("RobotState/lookaheadS", 0.35);
    private static final LoggedTunableNumber superPoopLookahead =
        new LoggedTunableNumber("RobotState/SuperPoopLookahead", 0.1);
    private static final LoggedTunableNumber closeShootingZoneFeet =
        new LoggedTunableNumber("RobotState/CloseShootingZoneFeet", 10.0);

    public AimingParameters getAimingParameters() {
        if (latestParameters != null) {
        // Cache previously calculated aiming parameters. Cache is invalidated whenever new
        // observations are added.
        return latestParameters;
        }

        Translation2d targetPosition = 
            AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).toTranslation2d();

        Pose2d predicitedRobotPose;
        if (DriverStation.isAutonomousEnabled()) {
            predicitedRobotPose = CatzRobotTracker.getInstance().getPredictedPose(autoLookahead.get(), autoLookahead.get());
        } else {
            predicitedRobotPose = CatzRobotTracker.getInstance().getPredictedPose(lookahead.get(), lookahead.get());
        }
        Logger.recordOutput("RobotState/AimingParameters/PredictedPose", predicitedRobotPose);

        Pose2d predictedRobotPoseFixed =
            new Pose2d(predicitedRobotPose.getTranslation(), new Rotation2d());

        Translation2d predictedVehicleToTargetTranslation = targetPosition.minus(predicitedRobotPose.getTranslation());
        Translation2d predictedVehicleFixedToTargetTranslation = targetPosition.minus(predictedRobotPoseFixed.getTranslation());

        // Calculations bassed off pose
        Rotation2d targetTurretDirection = predictedVehicleFixedToTargetTranslation.getAngle();
        double targetDistance = predictedVehicleToTargetTranslation.getNorm();

        //--------------------------------------------------------------------------------------------
        //  Calculate new turret target angle in deg based off:
        //    - Current robot position
        //    - Current robot rotation
        //---------------------------------------------------------------------------------  -----------
        double angle = Math.atan2(predictedVehicleToTargetTranslation.getY(), predictedVehicleToTargetTranslation.getX());
  
        //Logger.recordOutput("AutoAim/local turret target angle", angle);
  
        angle = CatzMathUtils.toUnitCircAngle(angle - predicitedRobotPose.getRotation().getRadians() - 3.14); 
        //Logger.recordOutput("AutoAim/global turret target angle", angle);
  
        double m_turretTargetDegree = Math.toDegrees(angle);    //Convert from radians to deg

        //Roll back if angle is past the softlimit
        if(m_turretTargetDegree > 180) {
          m_turretTargetDegree = m_turretTargetDegree - 360;
        } else if(m_turretTargetDegree < -180) {
          m_turretTargetDegree = m_turretTargetDegree + 360;
        }
        latestParameters =
            new AimingParameters(
                targetTurretDirection,
                Rotation2d.fromDegrees(0.0),
                targetDistance,
                new FlywheelSpeeds(0, 0));
        return latestParameters;
    }


    public record AimingParameters(
        Rotation2d driveHeading,
        Rotation2d armAngle,
        double effectiveDistance,
        FlywheelSpeeds flywheelSpeeds) {}

    public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {
        public static FlywheelSpeeds interpolate(FlywheelSpeeds t1, FlywheelSpeeds t2, double v) {
        double leftSpeed = MathUtil.interpolate(t1.leftSpeed(), t2.leftSpeed(), v);
        double rightSpeed = MathUtil.interpolate(t1.rightSpeed(), t2.rightSpeed(), v);
        return new FlywheelSpeeds(leftSpeed, rightSpeed);
        }
    }
}
