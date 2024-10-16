package frc.robot.CatzSubsystems.DriveAndRobotOrientation.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Utilities.LimelightHelpers;
import frc.robot.Utilities.LimelightHelpers.LimelightResults;

public class VisionIOLimeLight implements VisionIO {
    
    public String name;
    public boolean getTarget;
    private double[] lastData = new double[6];

    private int primaryTrackingApriltag;
    private Pose2d prevVisionPos = null;
    private Pose2d visionPose2d = null;
    private boolean badData = false;

     /**
     * Implements Limelight camera
     *
     * @param name Name of the limelight used, and should be configured in limelight software first
     */
    public VisionIOLimeLight(String name) {
        NetworkTableInstance.getDefault().getTable(name).getEntry("ledMode").setNumber(1);
        this.name = name;
        System.out.println("Limeilight " + name + " instantiated");
        
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        //load up raw apriltag values for distance calculations
        LimelightResults llresults = LimelightHelpers.getLatestResults(name);
        inputs.ty = NetworkTableInstance.getDefault().getTable(name).getEntry("ty").getDouble(0); //vertical offset from crosshair to target in degrees
        inputs.tx = NetworkTableInstance.getDefault().getTable(name).getEntry("tx").getDouble(0); //horizontal offset from crosshair to target
        inputs.tv = NetworkTableInstance.getDefault().getTable(name).getEntry("tv").getDouble(0); //whether the limelight has any vaild targets
        inputs.ta = NetworkTableInstance.getDefault().getTable(name).getEntry("ta").getDouble(0); //target area of the limelight from 0%-100%...how much does the apirltage take up on the frame
        inputs.primaryApriltagID = NetworkTableInstance.getDefault().getTable(name).getEntry("tid").getDouble(0);

        // collects pose information based off network tables and orients itself depending on alliance side
        //creating new pose3d object based of pose from network tables
        double[] visionPoseInfo = NetworkTableInstance.getDefault().getTable(name).getEntry("botpose_wpiblue").getDoubleArray(new double[7]); // this was 6 TODO
        inputs.tagCount = llresults.targetingResults.targets_Fiducials.length;
        inputs.maxDistance = llresults.targetingResults.botpose_avgdist;

        // set if the Limelight has a target to loggable boolean
        if (inputs.tv == 1) {
            inputs.hasTarget = true;
        } 
        else {
            inputs.hasTarget = false;
        }

        // calculates total latency using 7th table item in array 
        double latency = visionPoseInfo[6] / 1000; //data[6] or latency is recorded in ms; divide by 1000 to get s
        inputs.latency = latency;
        //shoves in new pose2d from pose3d object estimate depending on if new apriltag detected
            
        
        if (inputs.hasTarget && !name.equals("limelight-ramen")) {
            // sets input timestamp
            inputs.isNewVisionPose = true;
            
            //take new pose info from the limelight api
            visionPose2d = new Pose2d(visionPoseInfo[0],visionPoseInfo[1], new Rotation2d());

            //set a previous vision position depending on if we see an apriltag
            if(prevVisionPos == null){
                prevVisionPos = visionPose2d;
            }

            //logic for filtering bad estimates
            double error = visionPose2d.getTranslation().getDistance(prevVisionPos.getTranslation());
            
            //-------------------------------------------------------
            // error should be greater than 0 but less than 0.1 for vision estimates to be considered good
            //------------------------------------------------
            if(error == 0.0){
                //obtained a null error reading
                inputs.isNewVisionPose = false;
                return;
            }
            if(error > 0.1){

                badData = true;
            }

            if(!badData){
                //set new vision data if reading is considered good
                inputs.x = visionPose2d.getX();
                inputs.y = visionPose2d.getY();
                inputs.rotation = visionPose2d.getRotation().getRadians();
                inputs.timestamp = Timer.getFPGATimestamp() - latency;
            }
            
            prevVisionPos = visionPose2d;
            badData = false;           
        } 
        else {
            //limelight is not correct and doesn't currently have a target to look for
            inputs.isNewVisionPose = false;
            prevVisionPos = null;
        }

    } 

    @Override
    public String getName() {
        return name;
    }
}