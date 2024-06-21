package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import edu.wpi.first.math.geometry.*;


/***
 * CatzConstants
 * 
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 *         This class is where reusable constants are defined
 ***/
public final class CatzConstants {

  /**************************************************
   * Robot Modes
   *************************************************/
  public static final boolean tuningMode = true;
  public static final Mode currentMode = Mode.SIM;
  private static RobotType robotType = RobotType.SIM;

  public static final double LOOP_TIME = 0.02;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIM) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.SN2;
    }
    return robotType;
  }

  public static enum RobotType {
    /** Running on SN1. */
    SN1,
    /** Running on SN2. */
    SN2,
    /** Running on physics simulator */
    SIM
  }

  public static enum AllianceColor {
    Blue, Red
  }
  
  public static AllianceColor choosenAllianceColor = null;

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static final class OIConstants {

    public static final int XBOX_DRV_PORT = 0;
    public static final int XBOX_AUX_PORT = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.1;
    public static final double kOffPwr = 0.1;

  }

  public static final class VisionConstants {
    public static final double SPEAKER_HOOD_HEIGHT = 83.0;
    public static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);
    public static final Transform3d LIMELIGHT_OFFSET = new Transform3d(-Units.inchesToMeters(12),
        -Units.inchesToMeters(9), Units.inchesToMeters(20), new Rotation3d(0.0, 0.0, 180.0));
    public static final Transform3d LIMELIGHT_OFFSET_2 = new Transform3d(0.0, 0.0, 0.0, null);
  }

  public static final class TrajectoryConstants {
    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = 5;
  }

  // COLOR CONSTANTS::
  public static final class CatzColorConstants {
    public static final Color PHANTOM_SAPPHIRE = new Color(15, 25, 200);
  }

}