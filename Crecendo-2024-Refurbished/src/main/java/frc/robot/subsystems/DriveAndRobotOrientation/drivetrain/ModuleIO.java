package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface ModuleIO {
 @AutoLog
 static class ModuleIOInputs {
   public boolean isDriveMotorConnected;
   public double drivePositionUnits;
   public double driveVelocityRPS;
   public double driveAppliedVolts;
   public double driveSupplyCurrentAmps;
   public double driveTorqueCurrentAmps;
   
   public boolean isTurnMotorConnected;
   public double turnAbsoluteInitPosition;
   public Rotation2d turnPosition;
   public double turnVelocityRadsPerSec;
   public Rotation2d turnAbsolutePosition;
   public double turnBussVolts;
   public double turnSupplyCurrentAmps;
   public double[] odometryDrivePositionsMeters;
   public Rotation2d[] odometryTurnPositions;


 }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(ModuleIOInputs inputs) {}

 //---------------------------------------------------------------------------
 //   Drive Access Methods
 //---------------------------------------------------------------------------
 public default void runDrivePwrPercentIO(double drivePwrPercent) {}

 public default void runDriveVelocityRPSIO(double velocity, double feedForward) {}

 public default void setDriveBreakModeIO(boolean enable) {}

 public default void setDrvSensorPositionIO(double sensorpos) {}

 public default void setDriveSimPwrIO(double volts) {}

 public default void runCharacterization(double input) {}

 public default void setDrivePID(double kP, double kI, double kD) {}

 //---------------------------------------------------------------------------
 //   Steer Access Methods
 //---------------------------------------------------------------------------
 public default void runSteerPercentOutputIO(double SteerPwr) {}

 public default void runSteerPositionSetpoint(double currentAngleRad, double currentAngleRads) {}

 public default void setSteerBrakeModeIO(boolean enable) {}

 public default void setSteerSimPwrIO(double volts) {}

 public default void setTurnPID(double kP, double kI, double kD) {}

 //---------------------------------------------------------------------------
 //   Mag Enc Access Methods
 //---------------------------------------------------------------------------
 public default void resetMagEncoderIO() {}

}
