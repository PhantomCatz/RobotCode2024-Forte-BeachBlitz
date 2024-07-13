/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.*;
import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.driveConfig;
import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.moduleGainsAndRatios;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;
import frc.robot.utilities.Alert;
import frc.robot.utilities.CatzMathUtils;
import frc.robot.utilities.LoggedTunableNumber;
import frc.robot.utilities.CatzMathUtils.Conversions;

public class CatzSwerveModule {
    //Module delcaration block
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    // Module Strings for Logging
    private final String m_moduleName;

    // Global swerve module variables
    private SwerveModuleState m_swerveModuleState = null;

    // FeedFoward definment
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(moduleGainsAndRatios.ffkS(),
                                                                   moduleGainsAndRatios.ffkV(), 
                                                                   0.0);
    // Alerts                                                                               
    private final Alert driveMotorDisconnected;
    private final Alert steerMotorDisconnected;

    public CatzSwerveModule(ModuleConfig config, String moduleName) {
        this.m_moduleName = moduleName;
        // Run subsystem disconnect check
        if(DriveConstants.isDriveDisabled) { 
                io = new ModuleIONull();
                System.out.println("Module " + m_moduleName + " Unconfigured");
        } else {
            // Run Robot Mode hardware assignment
            switch (CatzConstants.hardwareMode) {
                case REAL: io = new ModuleIORealFoc(config);
                            System.out.println("Module " + m_moduleName + " Configured for Real");
                break;
                case REPLAY : io = new ModuleIORealFoc(config) {};
                            System.out.println("Module " + m_moduleName + " Configured for Replay simulation");
                break;
                case SIM: io = new ModuleIOSim(config);
                            System.out.println("Module " + m_moduleName + " Configured for WPILIB simulation");
                break;
                default : io = null;
                            System.out.println("Module " + m_moduleName + " Unconfigured");
                break;
            }
        }

        // Disconnected Alerts
        driveMotorDisconnected =
            new Alert(m_moduleName + " drive motor disconnected!", Alert.AlertType.WARNING);
        steerMotorDisconnected =
            new Alert(m_moduleName + " steer motor disconnected!", Alert.AlertType.WARNING);

        resetDriveEncs();
    } // -End of CatzSwerveModule Constructor

    public void periodic() {
        // Process and Log Module Inputs
        io.updateInputs(inputs);
        Logger.processInputs("Drive/M " + m_moduleName, inputs); 

        // Update ff and controllers
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> ff = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0),
            drivekS,
            drivekV);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setSteerPID(steerkP.get(), 0, steerkD.get()), steerkP, steerkD);

        // Display alerts
        driveMotorDisconnected.set(!inputs.isDriveMotorConnected);
        steerMotorDisconnected.set(!inputs.isSteerMotorConnected);

        // Logging
        debugLogsSwerve();
    } // -End of CatzSwerveModule Periodic 

    public void debugLogsSwerve(){
        Logger.recordOutput("Module " + m_moduleName + "/drive mps", m_swerveModuleState.speedMetersPerSecond);
        Logger.recordOutput("Module " + m_moduleName + "/current state", getModuleState());
        Logger.recordOutput("Module " + m_moduleName + "/angle error deg", Math.toDegrees(m_swerveModuleState.angle.getRadians()-getAbsEncRadians()));
        Logger.recordOutput("Module " + m_moduleName + "/currentmoduleangle rad", getAbsEncRadians());
        Logger.recordOutput("Module " + m_moduleName + "/targetmoduleangle rad", m_swerveModuleState.angle.getRadians());


        SmartDashboard.putNumber("absenctorad" + m_moduleName , getAbsEncRadians());
        SmartDashboard.putNumber("angle" + m_moduleName , getCurrentRotation().getDegrees());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setModuleAngleAndVelocity(SwerveModuleState state) { //TODO log variables actually used in calculations
        this.m_swerveModuleState       = state;
        double targetAngleRad          = state.angle.getRadians();
        double currentAngleRad         = getAbsEncRadians();

        // Run closed loop drive control
        io.runDriveVelocityRPSIO(
            Conversions.MPSToRPS(state.speedMetersPerSecond),
            ff.calculate(state.speedMetersPerSecond / driveConfig.wheelRadius())
        );
        // Run Closed Loop Steer Control
        io.runSteerPositionSetpoint(currentAngleRad, targetAngleRad);
    }

    //--------------------------------------------------------------------------------------------------------------------
    //          Drivetrain Power Setting methods
    //--------------------------------------------------------------------------------------------------------------------
    public void setSteerPower(double pwr) {
        io.runSteerPercentOutputIO(pwr);
    }

    public void setDriveVelocity(double velocity) {
        io.runDriveVelocityRPSIO(velocity, 0.0);
    }

    public void stopDriving() {
        io.runDrivePwrPercentIO(0.0);
    }

    //--------------------------------------------------------------------------------------------------------------------
    //          Module Util Methods
    //--------------------------------------------------------------------------------------------------------------------
    public void setBreakMode(boolean enable) {
        io.setSteerBrakeModeIO(enable);
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    /** optimze wheel angles before sending to setdesiredstate method for logging */
    public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
        SwerveModuleState optimizedState = CatzMathUtils.optimize(unoptimizedState, getCurrentRotation()); 
        return optimizedState;
    }

    //--------------------------------------------------------------------------------------------------------------------
    //          Module getters
    //--------------------------------------------------------------------------------------------------------------------
    public SwerveModuleState getModuleState() {
        double velocityMPS = CatzMathUtils.Conversions.RPSToMPS(inputs.driveVelocityRPS);
        
        return new SwerveModuleState(velocityMPS, getCurrentRotation());
    }

    public SwerveModuleState getModuleStateSetpoint() {
        return m_swerveModuleState;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        // seconds cancels out
        return CatzMathUtils.Conversions.RPSToMPS(inputs.drivePositionUnits);
    }

    public double getPositionRads() {
        return Units.rotationsToRadians(inputs.drivePositionUnits);
    }

    /** Get steer angle of module as {@link Rotation2d}. */
    public Rotation2d getAngle() {
        return inputs.steerAbsolutePosition;
    }

    /** Get velocity of drive wheel for characterization */
    public double getCharacterizationVelocityRadPerSec() {
        return Units.rotationsToRadians(getDrvVelocityRPS());
    }

    public double getDrvVelocityRPS() {
        return inputs.driveVelocityRPS;
    }

    /** Outputs the Rotation object of the module */
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }
    
    private double getAbsEncRadians() {
        //mag enc value should already have offset applied
        return inputs.steerAbsolutePosition.getRadians();
    }
}
