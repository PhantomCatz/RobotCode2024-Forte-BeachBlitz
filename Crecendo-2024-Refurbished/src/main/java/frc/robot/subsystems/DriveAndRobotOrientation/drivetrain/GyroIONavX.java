package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class GyroIONavX implements GyroIO 
{
    private final AHRS navX;

    public GyroIONavX() {
        navX = new AHRS(Port.kMXP, (byte) 200);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.gyroAngle = navX.getAngle();
      inputs.gyroYaw = (navX.getYaw());
      inputs.gyroRoll = navX.getRoll();
      inputs.gyroConnected = navX.isConnected();
      inputs.gyroAngleVel = navX.getRate();
      inputs.gyroAccelX   = navX.getWorldLinearAccelX();
      inputs.gyroAccelY   = navX.getWorldLinearAccelY();
    }

    @Override
    public void resetNavXIO(double angle){
        navX.reset();
        navX.setAngleAdjustment(angle);
    }

    @Override
    public void setAngleAdjustmentIO(double gyroAdjustment) {
        navX.setAngleAdjustment(gyroAdjustment);
    }

    @Override
    public double getAngleAdjustmentIO(){
        return navX.getAngleAdjustment();
    }


}

