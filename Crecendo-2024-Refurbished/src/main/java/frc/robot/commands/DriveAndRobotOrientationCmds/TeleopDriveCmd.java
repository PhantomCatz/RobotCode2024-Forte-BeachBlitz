package frc.robot.commands.DriveAndRobotOrientationCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;

public class TeleopDriveCmd extends Command {
  private CatzDrivetrain m_drivetrain;


  private Supplier<Double> m_supplierLeftJoyX;
  private Supplier<Double> m_supplierLeftJoyY;
  private Supplier<Double> m_supplierRightJoyX;
  private Supplier<Boolean> m_isFieldOrientedDisabled;

  //drive variables
  private double xVelocity;
  private double yVelocity;
  private double turningVelocity;

  private ChassisSpeeds chassisSpeeds;


  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX,
                        Supplier<Double> supplierLeftJoyY,
                        Supplier<Double> supplierRightJoyX,
                        Supplier<Boolean> supplierFieldOriented,
                        CatzDrivetrain drivetrain) {
    this.m_supplierLeftJoyX        = supplierLeftJoyX;
    this.m_supplierLeftJoyY        = supplierLeftJoyY;
    this.m_supplierRightJoyX       = supplierRightJoyX;
    this.m_isFieldOrientedDisabled = supplierFieldOriented;
    this.m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // obtain realtime joystick inputs with supplier methods
    xVelocity =       -m_supplierLeftJoyY.get(); 
    yVelocity =       -m_supplierLeftJoyX.get(); 
    turningVelocity =  m_supplierRightJoyX.get(); //alliance flip shouldn't change for turing speed when switching alliances

    // Flip Directions for left joystick if alliance is red
    if(CatzConstants.choosenAllianceColor == AllianceColor.Red) {
      xVelocity = -xVelocity;
      yVelocity = -yVelocity;
    }

    // Apply deadbands to prevent modules from receiving unintentional pwr
    xVelocity =       Math.abs(xVelocity) > XboxInterfaceConstants.kDeadband ? xVelocity * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    yVelocity =       Math.abs(yVelocity) > XboxInterfaceConstants.kDeadband ? yVelocity * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    turningVelocity = Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband ? turningVelocity * DriveConstants.driveConfig.maxAngularVelocity(): 0.0;

    Logger.recordOutput("Telopdrvcmd/CmdVelocityX", xVelocity);
    Logger.recordOutput("Telopdrvcmd/CmdVelocityY", yVelocity);


    // Construct desired chassis speeds
    if (m_isFieldOrientedDisabled.get()) {
        // Relative to robot
        chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, turningVelocity);
    } else {
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                            xVelocity, yVelocity, turningVelocity, CatzRobotTracker.getInstance().getRobotRotation()
                                                              );
    }

    //send new chassisspeeds object to the drivetrain
    m_drivetrain.drive(chassisSpeeds, true);
  }

  /*
  * For Debugging Purposes 
  * Keep them commmented ALWAYS if you are not using it 
  */
  public void debugLogsDrive(){
    //DEBUG
      // Logger.recordOutput("robot xspeed", xSpeed);
      // Logger.recordOutput("robot yspeed", ySpeed);
      // Logger.recordOutput("robot turnspeed", turningSpeed);
      // Logger.recordOutput("robot orientation", m_driveTrain.getRotation2d().getRadians());
      // Logger.recordOutput("chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
      // Logger.recordOutput("chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

