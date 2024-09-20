package frc.robot.Autonomous.commands;

import java.rmi.server.ExportException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzSubsystems.DriveAndRobotOrientation.CatzRobotTracker;

public class WaitUntilPassX extends Command{
    private double xCoord;
    private Command cmd;
    private boolean executing = false;
    private boolean done = false;

    private final double ERROR_RANGE = 0.2; //im pretty sure this is 20cm

    public WaitUntilPassX(double xCoord, Command command){
        this.xCoord = xCoord;
        this.cmd = command;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if(Math.abs(xCoord - CatzRobotTracker.getInstance().getEstimatedPose().getX()) <= ERROR_RANGE){
            executing = true;
            cmd.initialize();
        }

        if(executing){
            cmd.execute();
            if(cmd.isFinished()){
                done = true;
            }
        }
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
