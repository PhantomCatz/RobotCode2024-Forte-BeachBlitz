package frc.robot.subsystems.DriveAndRobotOrientation.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker.VisionFromAprilTagObservation;

 
/*
    Assume the Limelight is the front of the robot
*/
public class CatzVision extends SubsystemBase {

    // Implementation instantiation
    private final VisionIO[] cameras;
    public final VisionIOInputsAutoLogged[] inputs;

    private double targetID;
    private int acceptableTagID;
    private boolean useSingleTag = false;

    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);

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
        
        // For every limelight camera process vision with according logic
        for (int i = 0; i < inputs.length; i++) { //change to -1 if soba is installed
            // update and process new inputs[cameraNum] for camera
            cameras[i].updateInputs(inputs[i]);
            Logger.processInputs("Vsn/" + cameras[i].getName() + "/Inputs", inputs[i]);
           
            // Check when to process Vision Info
            if(cameras[i].getName().equals("limelight-ramen")) { 
                // Continue
            } else {
                if (inputs[i].hasTarget &&
                    inputs[i].maxDistance < LOWEST_DISTANCE) {
                    processVision(i);
                }
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

        // A vision updates to robot tracker
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