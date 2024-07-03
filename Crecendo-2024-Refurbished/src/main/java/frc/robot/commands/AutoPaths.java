package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;

public class AutoPaths {


    // Starting locations
    public static final Pose2d startingSource =
        new Pose2d(
            FieldConstants.startingLineX - 0.5,
            FieldConstants.Stage.podiumLeg.getY(),
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d startingCenter =
        new Pose2d(
            FieldConstants.startingLineX - 0.5,
            FieldConstants.StagingLocations.spikeTranslations[1].getY(),
            Rotation2d.fromDegrees(180.0));
    public static final Pose2d startingAmp =
        new Pose2d(
            FieldConstants.startingLineX - 0.5,
            FieldConstants.StagingLocations.spikeTranslations[2].getY(),
            Rotation2d.fromDegrees(180.0));


    public AutoPaths() {

        loadAutoPathsFromPathPlanner();
    }

    private void loadAutoPathsFromPathPlanner() {

    }
    
}
