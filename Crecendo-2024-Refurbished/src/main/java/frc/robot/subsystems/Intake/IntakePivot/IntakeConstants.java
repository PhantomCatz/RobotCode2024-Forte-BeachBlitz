// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

/** Add your docs here. */
public class IntakeConstants
{
    // -----------------------------------------------------------------------------------------------
    // Motor IDs
    // -----------------------------------------------------------------------------------------------
    public final static int PIVOT_MTR_ID = 12;
    public final static int ROLLER_MTR_ID = 10;

    // -----------------------------------------------------------------------------------------------
    // Kraken Configuration Constants
    // -----------------------------------------------------------------------------------------------
    public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 55;
    public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;

    // -----------------------------------------------------------------------------------------------
    // Rollers
    // -----------------------------------------------------------------------------------------------
    private final double ROLLERS_MTR_PWR_IN_GROUND = 0.8; //TBD - need to handle carpet and non-carpet value or code
                                                       // issue
    private final double ROLLERS_MTR_PWR_IN_SOURCE = 0.25;
    private final double ROLLERS_MTR_PWR_OUT_EJECT = -0.2; //0.2 // TBD fix top rooler before testing
    private final double ROLLERS_MTR_PWR_OUT_AMP_SCORE = 0.6;
    private final double ROLLERS_MTR_PWR_OUT_HANDOFF = -0.5;//-0.2;//-0.3;
    
    private static final boolean BEAM_IS_BROKEN     = true;
    private static final boolean BEAM_IS_NOT_BROKEN = false;
  
    private final double ROLLER_ADJUST_BACK    = -0.1;
    private final double ROLLER_ADJUST_FORWARD =  0.4;
    
    // -----------------------------------------------------------------------------------------------
    //
    // Pivot Definitions & Variables
    //
    // -----------------------------------------------------------------------------------------------
    // -----------------------------------------------------------------------------------------------
    // Pivot Gearbox Def
    // -----------------------------------------------------------------------------------------------
    private static final double INTAKE_PIVOT_DRIVEN_GEAR = 52.0;
    private static final double INTAKE_PIVOT_DRIVING_GEAR = 30.0;
  
    private static final double INTAKE_PIVOT_DRIVEN_SPROCKET = 32.0;
    private static final double INTAKE_PIVOT_DRIVING_SPROCKET = 16.0;
  
    private static final double MAX_PLANETARY_RATIO = 5.0;
  
    private static final double INTAKE_PIVOT_GEAR_RATIO = (INTAKE_PIVOT_DRIVEN_GEAR / INTAKE_PIVOT_DRIVING_GEAR) *
        (INTAKE_PIVOT_DRIVEN_SPROCKET / INTAKE_PIVOT_DRIVING_SPROCKET) *
        (MAX_PLANETARY_RATIO);
  
    private static final double INTAKE_PIVOT_MTR_REV_PER_DEG = INTAKE_PIVOT_GEAR_RATIO / 360.0;
  
    private static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG = 164.09;
  
    public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV = INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG
      * INTAKE_PIVOT_MTR_REV_PER_DEG;
    
    // -----------------------------------------------------------------------------------------------
    // Pivot Closed Loop Processing (PIDF, etc)
    // -----------------------------------------------------------------------------------------------
    public static final double PIVOT_PID_kP = 9.00; // 0.044
    public static final double PIVOT_PID_kI = 0.00; // 0.005
    public static final double PIVOT_PID_kD = 0.27;
  
    public final double PIVOT_FF_kS = 0.00;
    public final double PIVOT_FF_kG = 0.437;
    public final double PIVOT_FF_kV = 0.00;
    public final double PIVOT_FF_kA = 0.00;

    private double m_ffVolts = 0.0;

    private double m_targetPositionDeg = 0.0;
    private double m_nextTargetPositionDeg = INTAKE_NULL_DEG;
    private double m_currentPositionDeg = 0.0;
    private double m_previousTargetPositionDeg = 0.0;
  
    private boolean isIntakeInScoreAmp;
  
    private int m_iterationCounter;
  
    private double positionErrorDeg = 0.0;
    private double pivotVelRadPerSec = 0.0;
      
    // -----------------------------------------------------------------------------------------------
    // Control Mode Defs
    // -----------------------------------------------------------------------------------------------
    private final double SEMI_MANUAL_STEP_COEFFICIENT = 2.0;

    // -----------------------------------------------------------------------------------------------
    // Intake position defs & variables
    // -----------------------------------------------------------------------------------------------
    public static final double INTAKE_STOW_DEG           = 163.0;
    public static final double INTAKE_SOURCE_LOAD_DN_DEG = 30.0;
    public static final double INTAKE_SOURCE_LOAD_UP_DEG =  97.0; //with drivetrain inner rail to the
                                                               // bottom inner rail 7 1/4 inches
    public static final double INTAKE_AMP_SCORE_DN_DEG   =  95.6; //90.43; 
    public static final double INTAKE_HOARD_DEG          = 40.0;
    public static final double INTAKE_AMP_SCORE_DEG      = 80.0;
    public static final double INTAKE_GROUND_PICKUP_DEG  = -22.0; //-25.0;
    public static final double INTAKE_AMP_TRANSITION_DEG = -77.0; //TBD Change to -80 on sn2    
    public static final double INTAKE_MIN_ELEV_CLEARANCE_DEG = 100.0;
    public static final double INTAKE_TRANSITION_CHECK_DEG = -47.0; 
    private static final double INTAKE_NULL_DEG = -999.0;   
    private final static double INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG = 17.0;
    public final static double  INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV = 40.0;
    private final static double INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV = 32.0;    
    public final static double INTAKE_STOW_ELEV_CLEARED_DEG = 120.0;
    public static final double INTAKE_TURRET_CLEARANCE = 125.0;
}
