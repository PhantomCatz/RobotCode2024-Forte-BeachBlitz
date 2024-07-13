package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class GyroIONavX implements GyroIO           //TBD - can we put IO up front?  eg. ioGyro_NavX  may help file sorting
{
    private final AHRS navX;

    public GyroIONavX() {

        navX = new AHRS(Port.kMXP, (byte) 200);                                   //TBD = whats the 200 for?

        //navX.enableLogging(true); //TODO renable this when gyro is connected    //TBD - What does this do?  Do after reset?

        navX.reset(); // implicitly sets the gyro to 0 heading                    //TBD - Are we accounting for time to complete?
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) 
    {
      inputs.gyroAngle       = navX.getAngle(); //Acumulated Yaw
      inputs.gyroYawDegrees  = (navX.getYaw());         //TBD why is this one wrapped in extra parens?
      inputs.gyroRollDegrees = navX.getRoll();
      
      inputs.gyroConnected   = navX.isConnected();     //TBD How is this updated?

      inputs.gyroAngleVel    = navX.getRate();
      inputs.gyroAccelX      = navX.getWorldLinearAccelX();
      inputs.gyroAccelY      = navX.getWorldLinearAccelY();
    }

}