package frc.robot.CatzSubsystems.Shooter.ShooterFlywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {

  @AutoLog
  class FlywheelsIOInputs {
    public boolean isLeftMotorConnected = true;
    public boolean isRightMotorConnected = true;

    public double leftPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTorqueCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTorqueCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Update inputs */
  public default void updateInputs(FlywheelsIOInputs inputs) {}

  /** Run both motors at voltage */
  public default void runVolts(double leftVolts, double rightVolts) {}

  /** Stop both flywheels */
  public default void stop() {}

  /** Run left and right flywheels at velocity in rpm */
  public default void runVelocity(double leftRpm, 
                           double rightRpm, 
                           double leftFeedforward, 
                           double rightFeedforward) {}

  /** Config PID values for both motors */
  public default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  public default void setFF(double kS, double kV, double kA) {}

  /** Run left flywheels at voltage */
  public default void runCharacterizationLeft(double input) {}

  /** Run right flywheels at voltage */
  public default void runCharacterizationRight(double input) {}
}
