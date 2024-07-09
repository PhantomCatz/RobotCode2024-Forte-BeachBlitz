// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CatzConstants;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ElevatorConstants {
    public static final boolean isElevatorDisabled = true;

    public static final double MAXPLANETARY_GEAR_RATIO = 4.0 * 4.0;

    public static final double ELEVATOR_DRIVING_PULLEY = 24.0;
    public static final double ELEVATOR_DRIVEN_PULLEY  = 18.0;
  
    public static final double ELEVATOR_RATIO_STAGE_ONE = ELEVATOR_DRIVING_PULLEY/ELEVATOR_DRIVEN_PULLEY;
    public static final double ELEVATOR_GEAR_RATIO      = MAXPLANETARY_GEAR_RATIO * ELEVATOR_RATIO_STAGE_ONE;
    
    public static final double reduction = ELEVATOR_GEAR_RATIO;

    // Motor Choices
    public static final boolean leaderInverted = false;
    public static final double  minRotations = 0.0;
    public static final double  maxRotations = 117.0;

    public static final int leaderID =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> 25;
            default -> 11;
        };

    public static final int followerID =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> 25;
            default -> 11;
        };

    public static final double elevatorLength =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> Units.inchesToMeters(24.8);
            default -> Units.inchesToMeters(25.866);
        };

    public static final Translation2d elevatorOrigin = new Translation2d(-0.238, 0.298);

    public static final Gains gains =
        switch (CatzConstants.getRobotType()) {
            case SN2 -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);//TODO fix gains
            case SN1 -> new Gains(75.0, 0.0, 2.5, 0.0, 0.0, 0.0, 0.0);
            case SIM -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
        };

    public static final MotionMagicParameters motionMagicParameters =
        switch (CatzConstants.getRobotType()) {
            case SN2, SN1 -> new MotionMagicParameters(260, 400, 1600);
            case SIM -> new MotionMagicParameters(0.0, 0.0, 0.0);
        };

    public static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/Gains/kP", gains.kP());
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/Gains/kI", gains.kI());
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/Gains/kD", gains.kD());
    public static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Arm/Gains/kS", gains.ffkS());
    public static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Arm/Gains/kV", gains.ffkV());
    public static final LoggedTunableNumber kA =
        new LoggedTunableNumber("Arm/Gains/kA", gains.ffkA());
    public static final LoggedTunableNumber kG =
        new LoggedTunableNumber("Arm/Gains/kG", gains.ffkG());
    public static final LoggedTunableNumber mmCruiseVelocity =
        new LoggedTunableNumber("Arm/Gains/kV", motionMagicParameters.mmCruiseVelocity());
    public static final LoggedTunableNumber mmAcceleration =
        new LoggedTunableNumber("Arm/Gains/kA", motionMagicParameters.mmAcceleration());
    public static final LoggedTunableNumber mmJerk =
        new LoggedTunableNumber("Arm/Gains/kG", motionMagicParameters.mmJerk());
    public static final LoggedTunableNumber lowerLimitRotations =
        new LoggedTunableNumber("Arm/LowerLimitDegrees", minRotations);
    public static final LoggedTunableNumber upperLimitRotations =
        new LoggedTunableNumber("Arm/UpperLimitDegrees", maxRotations);


    //------------------------------------------------------------------------------------------------------------
    //      Elevator Record Types
    //------------------------------------------------------------------------------------------------------------
    public record Gains(
        double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}

    public record MotionMagicParameters(
        double mmCruiseVelocity, double mmAcceleration, double mmJerk) {}

}
