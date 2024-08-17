package frc.robot.Commands.DriveAndRobotOrientationCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.XboxInterfaceConstants;
import frc.robot.Subsystems.DriveAndRobotOrientation.CatzRobotTracker;
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.Subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;

public class TeleopDriveCmd extends Command {
  // Subsystem declaration
  private CatzDrivetrain m_drivetrain;

  // 
  private Supplier<Double> m_headingPctOutput_X;
  private Supplier<Double> m_headingPctOutput_Y;
  private Supplier<Double> m_angVelocityPctOutput;
  private Supplier<Boolean> m_orientation;

  //drive variables
  private double m_headingAndVelocity_X;
  private double m_headingAndVelocity_Y;
  private double turningVelocity;

  private ChassisSpeeds chassisSpeeds;


  //--------------------------------------------------------------------------------------
  //
  //  Teleop Drive Command Constructor
  // 
  //--------------------------------------------------------------------------------------
  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX,
                        Supplier<Double> supplierLeftJoyY,
                        Supplier<Double> angVelocityPctOutput,
                        Supplier<Boolean> orientation,
                        CatzDrivetrain drivetrain) {

    // Chassis magnatude and direction control
    this.m_headingPctOutput_X         = supplierLeftJoyX;
    this.m_headingPctOutput_Y         = supplierLeftJoyY;
    this.m_angVelocityPctOutput       = angVelocityPctOutput;

    // feild orientation disabling
    this.m_orientation                = orientation;

    // subsystem assignment
    this.m_drivetrain                 = drivetrain;

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // obtain realtime joystick inputs with supplier methods
    m_headingAndVelocity_X =       -m_headingPctOutput_Y.get(); 
    m_headingAndVelocity_Y =       -m_headingPctOutput_X.get(); 
    turningVelocity =               m_angVelocityPctOutput.get(); //alliance flip shouldn't change for turing speed when switching alliances

    // Flip Directions for left joystick if alliance is red
    if(CatzConstants.choosenAllianceColor == AllianceColor.Red) {
      m_headingAndVelocity_X = -m_headingAndVelocity_X;
      m_headingAndVelocity_Y = -m_headingAndVelocity_Y;
    }

    // Apply deadbands to prevent modules from receiving unintentional pwr due to joysticks having offset
    m_headingAndVelocity_X =       Math.abs(m_headingAndVelocity_X) > XboxInterfaceConstants.kDeadband ? m_headingAndVelocity_X * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    m_headingAndVelocity_Y =       Math.abs(m_headingAndVelocity_Y) > XboxInterfaceConstants.kDeadband ? m_headingAndVelocity_Y * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    turningVelocity =              Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband ? turningVelocity * DriveConstants.driveConfig.maxAngularVelocity(): 0.0;

    Logger.recordOutput("Telopdrvcmd/CmdVelocityX", m_headingAndVelocity_X);
    Logger.recordOutput("Telopdrvcmd/CmdVelocityY", m_headingAndVelocity_Y);
    Logger.recordOutput("Telopdrvcmd/CmdVelocityThetha", turningVelocity);


    // Construct desired chassis speeds
    if (m_orientation.get()) {
        // Relative to robot
        chassisSpeeds = new ChassisSpeeds(m_headingAndVelocity_X, m_headingAndVelocity_Y, turningVelocity);
    } else {
        // Relative to field
        chassisSpeeds = ChassisSpeeds
                            .fromFieldRelativeSpeeds(m_headingAndVelocity_X, 
                                                     m_headingAndVelocity_Y, 
                                                     turningVelocity, 
                                                     CatzRobotTracker.getInstance().getRobotRotation()
                            );
    }

    // Send new chassisspeeds object to the drivetrain
    m_drivetrain.drive(chassisSpeeds, true);
  }

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

