package frc.robot.commands.DriveAndRobotOrientationCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import frc.robot.CatzConstants;
import frc.robot.CatzConstants.AllianceColor;
import frc.robot.CatzConstants.XboxInterfaceConstants;

import frc.robot.subsystems.DriveAndRobotOrientation.CatzRobotTracker;

import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.CatzDrivetrain;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants;


public class TeleopDriveCmd extends Command {

  private CatzDrivetrain m_drivetrain;

  private Supplier<Double>  m_supplierLeftJoyX;     //headingPctOutput-X  Q: What is a supplier?  why does it have to be part of the name?
  private Supplier<Double>  m_supplierLeftJoyY;     //headingPctOutput-Y,
  private Supplier<Double>  m_supplierRightJoyX;    //angVelocityPctOutput

  private Supplier<Boolean> m_isFieldOrientedDisabled;  //TBD - Can we do orientation enum with FIELD_ORIENTED or ROBOT ORIENTED?  How does button work?  Is it a toggle?

  //drive variables
  private double xVelocity;         //TBD - this is heading & velocity,  headingVelocity_X
  private double yVelocity;
  private double turningVelocity;   //TBD - this is direction & velocity, should use angularVelocity to be consistent

  private ChassisSpeeds chassisSpeeds;



  //--------------------------------------------------------------------------------------
  //
  //  Teleop Drive Command    //TBD - Is there an Auton Drive cmd?  There can be multiple cmds in a file correct?  If so default methods should go in this section
  //
  //--------------------------------------------------------------------------------------
  public TeleopDriveCmd(Supplier<Double>  supplierLeftJoyX,
                        Supplier<Double>  supplierLeftJoyY,
                        Supplier<Double>  supplierRightJoyX,
                        Supplier<Boolean> supplierFieldOriented,  //TBD - We're passing in Field or Robot oriented correct?
                        CatzDrivetrain    drivetrain) 
  {
    this.m_supplierLeftJoyX        = supplierLeftJoyX;
    this.m_supplierLeftJoyY        = supplierLeftJoyY;
    this.m_supplierRightJoyX       = supplierRightJoyX;

    this.m_isFieldOrientedDisabled = supplierFieldOriented;
    
    this.m_drivetrain              = drivetrain;

    addRequirements(m_drivetrain);
  }


  public TeleopDriveCmd(Supplier<Double>  headingPctOutput-X,
                        Supplier<Double>  headingPctOutput-Y,
                        Supplier<Double>  angVelocityPctOutput,
                        Supplier<Boolean> orientation,
                        CatzDrivetrain    drivetrain) 
  {
    this.m_headingPctOutput_X   = headingPctOutput_X.get();    //TBD - Can we do this
    this.m_headingPctOutput_Y   = headingPctOutput_Y.get();
    this.m_angVelocityPctOutput = angVelocityPctOutput.get();

    this.m_orientation          = orientation.get();
    
    this.m_drivetrain           = drivetrain;

    addRequirements(m_drivetrain);
  }


  //------------------------------------------------------------------------------------------------
  //
  //  Default Methods - This only works if only 1 cmd per file
  //
  //------------------------------------------------------------------------------------------------
  @Override
  public void initialize() {}



  @Override
  public void execute() 
  {
    // obtain realtime joystick inputs with supplier methods
    xVelocity       = -m_supplierLeftJoyY.get();    //TBD - Why are x & y flipped?  Why inverted?  Why are we doing this?  Why not store directly?
    yVelocity       = -m_supplierLeftJoyX.get();
    turningVelocity =  m_supplierRightJoyX.get(); //alliance flip shouldn't change for turing speed when switching alliances  
                                                          //TBD - Shouldn't??? Has this been verified?  Shouldn't orientation come into play here?

    // Flip Directions for left joystick if alliance is red
    if(CatzConstants.choosenAllianceColor == AllianceColor.Red) {   //TBD this is kind of scary - this means we MUST develop from BLUE perspective consistently
                                                                    //    Is this for teleop or auton or Both??  What about rotation?
      xVelocity = -xVelocity;
      yVelocity = -yVelocity;
    }

    // Apply deadbands to prevent modules from receiving unintentional pwr
    xVelocity =       Math.abs(xVelocity) > XboxInterfaceConstants.kDeadband ? xVelocity * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    yVelocity =       Math.abs(yVelocity) > XboxInterfaceConstants.kDeadband ? yVelocity * DriveConstants.driveConfig.maxLinearVelocity(): 0.0;
    turningVelocity = Math.abs(turningVelocity) > XboxInterfaceConstants.kDeadband ? turningVelocity * DriveConstants.driveConfig.maxAngularVelocity(): 0.0;


    // Construct desired chassis speeds
    if (m_orientation.get()) {
      // Relative to robot
      chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, turningVelocity);   //TBD Why does only this one have new?
    } else {
        // Field Oriented
        chassisSpeeds =     ChassisSpeeds.fromFieldRelativeSpeeds(
                                            xVelocity, yVelocity, turningVelocity, CatzRobotTracker.getInstance().getRobotRotation()
                                                              );
    }

    //send new chassisspeeds object to the drivetrain
    m_drivetrain.driveWithDiscretizeKinematics(chassisSpeeds);

  }   //End of execuite() ORIG


  @Override
  public void execute() 
  {
    //----------------------------------------------------------------------------------------------
    // Flip Directions for left joystick if alliance is BLUE
    //----------------------------------------------------------------------------------------------
    if(CatzConstants.choosenAllianceColor == AllianceColor.Blue)
    {
      this.m_headingAndVelocity_X = -this.m_headingAndVelocity_X;
      this.m_headingAndVelocity_Y = -this.m_headingAndVelocity_Y;
    }

    //----------------------------------------------------------------------------------------------
    //  Apply deadbands to prevent modules from receiving unintentional pwr due to joystick axis
    //  having an offset (not trimmed to zero).  If in deadband, zero velocity value, else set to
    //  % of max velocity
    //----------------------------------------------------------------------------------------------
    headingVelocity_X = Math.abs(headingPctOutput_X)   > XboxInterfaceConstants.kDeadband ? headingPctOutput_X   * DriveConstants.driveConfig.maxLinearVelocity (): 0.0;
    headingVelocity_Y = Math.abs(headingPctOutput_Y)   > XboxInterfaceConstants.kDeadband ? headingPctOutput_Y   * DriveConstants.driveConfig.maxLinearVelocity (): 0.0;

    angularVelocity   = Math.abs(angVelocityPctOutput) > XboxInterfaceConstants.kDeadband ? angVelocityPctOutput * DriveConstants.driveConfig.maxAngularVelocity(): 0.0;

    //----------------------------------------------------------------------------------------------
    // Construct desired chassis speeds
    //----------------------------------------------------------------------------------------------
    if (m_orientation.get() == ROBOT_ORIENTED) {
        chassisSpeeds = new ChassisSpeeds(headingVelocity_X, headingVelocity_Y, angularVelocity);
    } else {
        // Relative to field
        chassisSpeeds =     ChassisSpeeds.fromFieldRelativeSpeeds(this.m_headingAndVelocity_X, this.m_headingAndVelocity_Y, 
                                                                  angularVelocity, 
                                                                  CatzRobotTracker.getInstance().getRobotRotation()       );    //TBD - Need explanation
    }

    m_drivetrain.driveWithDiscretizeKinematics(chassisSpeeds);    //send new chassisspeeds object to the drivetrain

  }   //End of execuite()



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  //--------------------------------------------------------------------------------------
  //
  //  TBD Methods
  //
  //--------------------------------------------------------------------------------------
  /*
  * For Debugging Purposes 
  * Keep them commmented ALWAYS if you are not using it 
  */
  public void debugLogsDrive(){
    //DEBUG
      // Logger.recordOutput("robot xspeed",      xSpeed);
      // Logger.recordOutput("robot yspeed",      ySpeed);
      // Logger.recordOutput("robot turnspeed",   turningSpeed);
      
      // Logger.recordOutput("robot orientation", m_driveTrain.getRotation2d().getRadians());

      // Logger.recordOutput("chassisspeed x speed mtr sec", chassisSpeeds.vxMetersPerSecond);
      // Logger.recordOutput("chassisspeed y speed mtr sec", chassisSpeeds.vyMetersPerSecond);
  }

}