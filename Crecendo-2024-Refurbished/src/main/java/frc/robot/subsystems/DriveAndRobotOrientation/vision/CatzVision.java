package frc.robot.subsystems.DriveAndRobotOrientation.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants.VisionConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker.VisionFromAprilTagObservation;

 
/*
    Assume the Limelight is the front of the robot
*/
public class CatzVision extends SubsystemBase {

    //io block
    private final VisionIO[] cameras;
    public final VisionIOInputsAutoLogged[] inputs;

    private final List<CatzVision.PoseAndTimestamp> results = new ArrayList<>(); //in a list to account for multiple cameras

    private double targetID;
    private int acceptableTagID;
    private boolean useSingleTag = false;

    //constructor for vision subsystem that creates new vision input objects for each camera set in the singleton implementation
    public CatzVision(VisionIO[] cameras) {
        this.cameras = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];

        for(int i = 0; i < cameras.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

    }

    @Override
    public void periodic() {

        // clear results from last periodic
        results.clear();
        
        //for every limlight camera process vision with according logic
        for (int i = 0; i < inputs.length; i++) { //change to -1 if soba is installed
            // update and process new inputs[cameraNum] for camera
            
            cameras[i].updateInputs(inputs[i]);
            Logger.processInputs("Vsn/" + cameras[i].getName() + "/Inputs", inputs[i]);
                    
            //checks for when to process vision
            if (inputs[i].hasTarget &&
                inputs[i].maxDistance < VisionConstants.LOWEST_DISTANCE) { //TBD get rid of this?
                processVision(i);
            }
        }        

        //DEBUG
        //Logger.recordOutput("Vision/ResultCount", results.size());
    }

    static int camNum;
    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs[cameraNum

        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                                        inputs[cameraNum].y, 
                                        new Rotation2d(inputs[cameraNum].rotation));

        // add the new pose to a list
       // CatzRobotTracker.getInstance().addVisionObservation();
        results.add(new PoseAndTimestamp(currentPose, 
                                         inputs[cameraNum].timestamp, 
                                         inputs[cameraNum].tagCount, 
                                         inputs[cameraNum].ta, 
                                         cameras[cameraNum].getName(), 
                                         inputs[cameraNum].hasTarget));

        CatzRobotTracker.getInstance()
                            .addVisionObservation(
                                new VisionFromAprilTagObservation(inputs[cameraNum].timestamp, 
                                                                  currentPose, 
                                                                  inputs[cameraNum].primaryApriltagID, 
                                                                  inputs[cameraNum].hasTarget,
                                                                  0.0, 
                                                                  inputs[cameraNum].ta, 
                                                                   cameras[cameraNum].getName()));

        camNum = cameraNum;
    }

    //Returns the last recorded pose in a list
    public List<CatzVision.PoseAndTimestamp> getVisionOdometry() {
        return results;
    }

    //Inner class to record a pose and its timestamp
    public static class PoseAndTimestamp {
        private Pose2d pose;
        private double timestamp;
        private int numOfTagsVisible;
        private double avgArea;
        private String name;
        private boolean hasTarget;

        public PoseAndTimestamp(Pose2d pose, double timestamp, int numOfTagsVisible, double avgArea, String name, boolean hasTarget) {
            this.pose = pose;
            this.timestamp = timestamp;
            this.numOfTagsVisible = numOfTagsVisible;
            this.avgArea = avgArea;
            this.name = name;
            this.hasTarget = hasTarget;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public int getNumOfTagsVisible(){
            return numOfTagsVisible;
        }

        public double getAvgArea(){
            return avgArea;
        }

        public String getName(){
            return name;
        }

        public boolean hasTarget(){
            return hasTarget;
        }
    }

    //------------------------------------------------------------------------
    // Util
    //------------------------------------------------------------------------
    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    
    public double getOffsetX(int cameraNum) {
        return inputs[cameraNum].tx;
    }

    public double getOffsetY(int cameraNum) {
        return inputs[cameraNum].ty;
    }

    public double getAprilTagID(int cameraNum) {
        return inputs[cameraNum].primaryApriltagID;
    }

    public int getCameraNum() {
        return camNum;
    }


}