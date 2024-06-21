package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface ModuleIO {
 @AutoLog
 static class ModuleIOInputs {
   public double driveMtrVelocity = 0.0;
   public double driveMtrSensorPosition = 0.0;
   public double magEncoderValue = 0.0;

   public double drivePositionRads;
   public double driveVelocityRadsPerSec;
   public double driveAppliedVolts;
   public double driveSupplyCurrentAmps;
   
   
   public double turnAbsoluteInitPosition;
   public Rotation2d turnPosition;
   public double turnVelocityRadsPerSec;
   public Rotation2d turnAbsolutePosition;
   public double turnAppliedVolts;
   public double turnSupplyCurrentAmps;
   public double[] odometryDrivePositionsMeters;
   public Rotation2d[] odometryTurnPositions;


 }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(ModuleIOInputs inputs) {}

 public default void setDrivePwrPercentIO(double drivePwrPercent) {}

 public default void setDriveVelocityIO(double velocity, double feedForward) {}

 public default void setSteerPwrIO(double SteerPwr) {}

 public default void runTurnPositionSetpoint(double angleRads) {}

 public default void setSteerBrakeModeIO(boolean enable) {}

 public default void setDriveBreakModeIO(boolean enable) {}

 public default void setDrvSensorPositionIO(double sensorpos) {}

 public default void reverseDriveIO(boolean enable) {}

 public default void setDriveSimPwrIO(double volts) {}

 public default void setSteerSimPwrIO(double volts) {}

 public default void resetMagEncoderIO() {}

 public default void runCharacterization(double input) {}

 public default void setDrivePID(double kP, double kI, double kD) {}

 public default void setTurnPID(double kP, double kI, double kD) {}


}
