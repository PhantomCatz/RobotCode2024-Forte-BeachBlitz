/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.driveConfig;
import static frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.moduleGainsAndRatios;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;
import frc.robot.util.Alert;
import frc.robot.util.CatzMathUtils;
import frc.robot.util.CatzMathUtils.Conversions;
import frc.robot.util.LoggedTunableNumber;

public class CatzSwerveModule {
    //Module delcaration block
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private static final LoggedTunableNumber drivekP =
        new LoggedTunableNumber("Drive/Module/DrivekP", moduleGainsAndRatios.drivekP());
    private static final LoggedTunableNumber drivekD =
        new LoggedTunableNumber("Drive/Module/DrivekD", moduleGainsAndRatios.drivekD());
    private static final LoggedTunableNumber drivekS =
        new LoggedTunableNumber("Drive/Module/DrivekS", moduleGainsAndRatios.ffkS());
    private static final LoggedTunableNumber drivekV =
        new LoggedTunableNumber("Drive/Module/DrivekV", moduleGainsAndRatios.ffkV());
    private static final LoggedTunableNumber turnkP =
        new LoggedTunableNumber("Drive/Module/TurnkP", moduleGainsAndRatios.turnkP());
    private static final LoggedTunableNumber turnkD =
        new LoggedTunableNumber("Drive/Module/TurnkD", moduleGainsAndRatios.turnkD());

    private static final String[] moduleNames = new String[] {"FL", "BL", "BR", "FR"};


    //global swerve module variables
    private int m_index;
    private SwerveModuleState m_swerveModuleState = null;

    // FeedFoward definment
    private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(moduleGainsAndRatios.ffkS(), moduleGainsAndRatios.ffkV(), 0.0);

    // Alerts
    private final Alert driveMotorDisconnected;
    private final Alert turnMotorDisconnected;


    public CatzSwerveModule(ModuleConfig config, int index) {
        m_index = index;

        // Run subsystem disconnect check
        if(DriveConstants.isDriveDisabled) { //TODO add extra robot enviroment //TODO have discussion on mode and states definement
                io = new ModuleIONull();
                System.out.println("Module " + moduleNames[m_index] + " Unconfigured");
        } else {
            // Run Robot Mode hardware assignment
            switch (CatzConstants.hardwareMode) {
                case REAL: io = new ModuleIORealFoc(config);
                            System.out.println("Module " + moduleNames[m_index] + " Configured for Real");
                break;
                case REPLAY : io = new ModuleIORealFoc(config) {};
                            System.out.println("Module " + moduleNames[m_index] + " Configured for Replay simulation");
                break;
                case SIM: io = new ModuleIOSim(config);
                            System.out.println("Module " + moduleNames[m_index] + " Configured for WPILIB simulation");
                break;
                default : io = null;
                            System.out.println("Module " + moduleNames[m_index] + " Unconfigured");
                break;
            }
        }

        // Disconnected Alerts
        driveMotorDisconnected =
            new Alert(moduleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
        turnMotorDisconnected =
            new Alert(moduleNames[index] + " turn motor disconnected!", Alert.AlertType.WARNING);

        resetDriveEncs();
    } // -End of CatzSwerveModule Constructor

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drv/M " + moduleNames[m_index], inputs); 

        // Update ff and controllers
        LoggedTunableNumber.ifChanged(
            hashCode(),
            () -> ff = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0),
            drivekS,
            drivekV);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
        LoggedTunableNumber.ifChanged(
            hashCode(), () -> io.setTurnPID(turnkP.get(), 0, turnkD.get()), turnkP, turnkD);

        // Display alerts
        driveMotorDisconnected.set(!inputs.isDriveMotorConnected);
        turnMotorDisconnected.set(!inputs.isTurnMotorConnected);

        // Logging
        debugLogsSwerve();
    }

    //-----------------------------------------LOGS----------------------------------------------
    /**
    * For Debugging Purposes 
    * Keep them commmented ALWAYS if you are not using it 
    */
    public void debugLogsSwerve(){
        Logger.recordOutput("Module/absenctorad" + moduleNames[m_index] , getAbsEncRadians());
        Logger.recordOutput("Module/angle" + moduleNames[m_index] , getCurrentRotation().getDegrees());
        Logger.recordOutput("Module " + moduleNames[m_index] + "/drive applied volts", inputs.driveAppliedVolts);


        SmartDashboard.putNumber("absenctorad" + moduleNames[m_index] , getAbsEncRadians());
        SmartDashboard.putNumber("angle" + moduleNames[m_index] , getCurrentRotation().getDegrees());
    }

    //----------------------------------------Setting pwr methods-------------------------------
    public void setSteerPower(double pwr) {
        io.runSteerPercentOutputIO(pwr);
    }

    public void setDriveVelocity(double velocity) {
        io.runDriveVelocityRPSIO(velocity, 0.0);
    }

    public void stopDriving() {
        io.runDrivePwrPercentIO(0.0);
    }

    //----------------------------------Util Methods catzswerve------------------------
    public void setBreakMode(boolean enable) {
        io.setSteerBrakeModeIO(enable);
    }

    public double getPositionRads() {
        return Units.rotationsToRadians(inputs.drivePositionUnits);
    }

    /** Get turn angle of module as {@link Rotation2d}. */
    public Rotation2d getAngle() {
        return inputs.turnAbsolutePosition;
    }

    /** Get velocity of drive wheel for characterization */
    public double getCharacterizationVelocityRadPerSec() {
        return Units.rotationsToRadians(getDrvVelocityRPS());
    }

    public double getDrvVelocityRPS() {
        return inputs.driveVelocityRPS;
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }
    
    private double getAbsEncRadians() {
        //mag enc value should already have offset applied
        return inputs.turnAbsolutePosition.getRadians();
    }
    
    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        this.m_swerveModuleState = state;
        double targetAngleRad          = state.angle.getRadians();
        double currentAngleRad         = getAbsEncRadians();
        // Run closed loop drive control

        double wheelTorqueNm = 0.0;//torqueFF.speedMetersPerSecond; // Using SwerveModuleState for torque for easy logging //TODO finish off
        io.runDriveVelocityRPSIO(
            Conversions.MPSToRPS(state.speedMetersPerSecond),
            ff.calculate(state.speedMetersPerSecond / driveConfig.wheelRadius())
                + ((wheelTorqueNm / moduleGainsAndRatios.driveReduction()) * moduleGainsAndRatios.ffkT()));

        if(CatzConstants.getRobotType() == CatzConstants.RobotID.SN_TEST) {
            io.runSteerPositionSetpoint(0.0, targetAngleRad);
        } else {
            io.runSteerPositionSetpoint(currentAngleRad, targetAngleRad);
        }

        //DEBUG
        Logger.recordOutput("Module " + moduleNames[m_index] + "/angle error deg", Math.toDegrees(targetAngleRad-currentAngleRad));
        Logger.recordOutput("Module " + moduleNames[m_index] + "/drive mps", state.speedMetersPerSecond);
        Logger.recordOutput("Module " + moduleNames[m_index] + "/current state", getModuleState());
        Logger.recordOutput("Module " + moduleNames[m_index] + "/currentmoduleangle rad", currentAngleRad);
        Logger.recordOutput("Module " + moduleNames[m_index] + "/targetmoduleangle rad", targetAngleRad);
    }

    /** optimze wheel angles before sending to setdesiredstate method for logging */
    public SwerveModuleState optimizeWheelAngles(SwerveModuleState unoptimizedState) {
        SwerveModuleState optimizedState = CatzMathUtils.optimize(unoptimizedState, getCurrentRotation()); 
        return optimizedState;
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

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
}
