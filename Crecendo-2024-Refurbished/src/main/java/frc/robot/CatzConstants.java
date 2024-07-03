package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

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
  public static final RobotEnviroment robotEnviroment = RobotEnviroment.PRACTICE;
  public static final HardwareMode hardwareMode = HardwareMode.SIM;
  private static RobotType robotType = RobotType.SIM;
  
  public static AllianceColor choosenAllianceColor = null;
  public static boolean disableHAL = false;

  public static final double LOOP_TIME = 0.02;


  public static enum RobotEnviroment {
    TUNING, //In PID enviroment with logged tunable numbers
    PRACTICE, //Driver Practice + Testing
    COMPETITION //Competition Setting
  }

  public static enum HardwareMode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static RobotType getRobotType() {
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

  // COLOR CONSTANTS::
  public static final class CatzColorConstants {
    public static final Color PHANTOM_SAPPHIRE = new Color(15, 25, 200);
  }

}