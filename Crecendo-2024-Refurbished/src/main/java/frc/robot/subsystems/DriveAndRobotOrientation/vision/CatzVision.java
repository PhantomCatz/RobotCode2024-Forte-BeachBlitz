package frc.robot.Subsystems.DriveAndRobotOrientation.vision;
    
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Subsystems.DriveAndRobotOrientation.CatzRobotTracker.VisionFromAprilTagObservation;
import frc.robot.subsystems.DriveAndRobotOrientation.vision.VisionIOInputsAutoLogged;

public class CatzVision extends SubsystemBase {

    // Hardware IO declaration
    private final VisionIO[] cameras;
    public final VisionIOInputsAutoLogged[] inputs;

    // MISC variables
    private double targetID;
    private int acceptableTagID;
    private boolean useSingleTag = false;
    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);

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
        for (int i = 0; i < inputs.length; i++) {
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
                                        new Rotation2d(inputs[cameraNum].rotation)
                             );

        // A vision updates to robot tracker
        CatzRobotTracker.getInstance()
                            .addVisionObservation(
                                new VisionFromAprilTagObservation(inputs[cameraNum].timestamp, 
                                                                  currentPose, 
                                                                  inputs[cameraNum].primaryApriltagID, 
                                                                  inputs[cameraNum].hasTarget,
                                                                  0.0, 
                                                                  inputs[cameraNum].ta, 
                                                                  cameras[cameraNum].getName())
                            );

        camNum = cameraNum;
    }


    //------------------------------------------------------------------------
    //
    //      Vision util methods 
    //
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