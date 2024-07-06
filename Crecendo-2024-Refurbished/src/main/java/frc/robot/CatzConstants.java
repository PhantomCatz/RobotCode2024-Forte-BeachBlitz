package frc.robot;

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
  public static final Mode currentMode = Mode.REAL;
  private static RobotType robotType = RobotType.SN2;

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

}