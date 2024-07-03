// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

public class IntakeConstants
{
  // -----------------------------------------------------------------------------------------------
  // 
  // Motor IDs
  // 
  // -----------------------------------------------------------------------------------------------
  public final static int PIVOT_MTR_ID = 12;
  public final static int ROLLER_MTR_ID = 10;

  // -----------------------------------------------------------------------------------------------
  // 
  // Kraken Configuration Constants
  // 
  // -----------------------------------------------------------------------------------------------
  public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 55;
  public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 55;
  public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;

  // -----------------------------------------------------------------------------------------------
  // 
  // Intake Pivot
  // 
  // -----------------------------------------------------------------------------------------------

  // -----------------------------------------------------------------------------------------------
  // Gearbox Definitions
  // -----------------------------------------------------------------------------------------------
  public static final double INTAKE_PIVOT_DRIVEN_GEAR = 52.0;
  public static final double INTAKE_PIVOT_DRIVING_GEAR = 30.0;

  public static final double INTAKE_PIVOT_DRIVEN_SPROCKET = 32.0;
  public static final double INTAKE_PIVOT_DRIVING_SPROCKET = 16.0;

  public static final double MAX_PLANETARY_RATIO = 5.0;

  public static final double INTAKE_PIVOT_GEAR_RATIO = (INTAKE_PIVOT_DRIVEN_GEAR / INTAKE_PIVOT_DRIVING_GEAR) *
      (INTAKE_PIVOT_DRIVEN_SPROCKET / INTAKE_PIVOT_DRIVING_SPROCKET) *
      (MAX_PLANETARY_RATIO);

  public static final double INTAKE_PIVOT_MTR_REV_PER_DEG = INTAKE_PIVOT_GEAR_RATIO / 360.0;

  public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG = 164.09;

  public static final double INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV = INTAKE_PIVOT_MTR_POS_OFFSET_IN_DEG
    * INTAKE_PIVOT_MTR_REV_PER_DEG;
    
  // -----------------------------------------------------------------------------------------------
  // Control Mode Defs
  // -----------------------------------------------------------------------------------------------
  public final double SEMI_MANUAL_STEP_COEFFICIENT = 2.0;

  public static enum IntakeControlState {
    AUTO,
    SEMI_MANUAL,
    FULL_MANUAL,
  }

  // -----------------------------------------------------------------------------------------------
  // Position Definitions & Variables
  // -----------------------------------------------------------------------------------------------
  public static final double INTAKE_STOW_DEG           = 163.0;
  public static final double INTAKE_AMP_SCORE_DEG      = 80.0;
  public static final double INTAKE_GROUND_PICKUP_DEG  = -22.0; //-25.0;
  
  public static final double INTAKE_SOURCE_LOAD_DN_DEG = 30.0;
  public static final double INTAKE_SOURCE_LOAD_UP_DEG =  97.0; //with drivetrain inner rail to the
                                                                // bottom inner rail 7 1/4 inches
  public static final double INTAKE_AMP_SCORE_DN_DEG   =  95.6; //90.43; 
  public static final double INTAKE_HOARD_DEG          = 40.0;
  public static final double INTAKE_AMP_TRANSITION_DEG = -77.0; //TBD Change to -80 on sn2

  public static final double INTAKE_MIN_ELEV_CLEARANCE_DEG = 100.0;
  public static final double INTAKE_TRANSITION_CHECK_DEG = -47.0; 
  public static final double INTAKE_NULL_DEG = -999.0;   
  public final static double INTAKE_TURRET_MAX_ANGLE_FOR_HANDOFF_DEG = 17.0;
  public final static double INTAKE_ELEV_MAX_HEIGHT_FOR_INTAKE_STOW_REV = 40.0;
  public final static double INTAKE_ELEV_MIN_HEIGHT_FOR_AMP_TRANS_REV = 32.0;    
  public final static double INTAKE_STOW_ELEV_CLEARED_DEG = 120.0;
  public static final double INTAKE_TURRET_CLEARANCE = 125.0;

  // -----------------------------------------------------------------------------------------------
  // 
  // Intake Rollers
  // 
  // -----------------------------------------------------------------------------------------------

  // -----------------------------------------------------------------------------------------------
  // Motor Power PercentOutput Constants
  // -----------------------------------------------------------------------------------------------
  public final static double ROLLERS_MTR_PWR_IN_GROUND = 0.8; //TBD - need to handle carpet and non-carpet value or code
                                                       // issue

  public static final double ROLLERS_MTR_PWR_OUT_EJECT = -0.2; //0.2 // TBD fix top rooler before testing
  public static final double ROLLERS_MTR_PWR_OUT_AMP_SCORE = 0.6;
  public static final double ROLLERS_MTR_PWR_OUT_HANDOFF = -0.5;//-0.2;//-0.3;

  public final double ROLLER_ADJUST_BACK    = -0.1;
  public final double ROLLER_ADJUST_FORWARD =  0.4;
  
  // -----------------------------------------------------------------------------------------------
  // Beam Break Definitions and Variables
  // -----------------------------------------------------------------------------------------------
  public static final boolean BEAM_IS_BROKEN     = true;
  public static final boolean BEAM_IS_NOT_BROKEN = false;
  
  public boolean m_desiredBeamBreakState;
}
