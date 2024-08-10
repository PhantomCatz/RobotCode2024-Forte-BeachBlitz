// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRollers;

public class IntakeConstants
{
  // -----------------------------------------------------------------------------------------------
  // 
  // Intake Rollers
  // 
  // -----------------------------------------------------------------------------------------------

  // -----------------------------------------------------------------------------------------------
  // Motor ID
  // -----------------------------------------------------------------------------------------------
  public final static int ROLLER_MTR_ID = 10;

  // -----------------------------------------------------------------------------------------------
  // Configuration Constants
  // -----------------------------------------------------------------------------------------------
  public static final double  INTAKE_ROLLER_CURRENT_LIMIT_AMPS            = 55.0;
  public static final double  INTAKE_ROLLER_CURRENT_LIMIT_TRIGGER_AMPS    = 55.0;
  public static final double  INTAKE_ROLLER_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  public static final boolean INTAKE_ROLLER_ENABLE_CURRENT_LIMIT          = true;

  // -----------------------------------------------------------------------------------------------
  // Motor Power PercentOutput Constants
  // -----------------------------------------------------------------------------------------------
  public final static double ROLLERS_MTR_PWR_IN_GROUND = 0.8; //TBD - need to handle carpet and non-carpet value or code
                                                              // issue

  public static final double ROLLERS_MTR_PWR_OUT_EJECT     = -0.2; //0.2 // TBD fix top roller before testing
  public static final double ROLLERS_MTR_PWR_OUT_AMP_SCORE = 0.6;
  public static final double ROLLERS_MTR_PWR_OUT_HANDOFF   = -0.5;//-0.2;//-0.3;

  public final double ROLLER_ADJUST_BACK    = -0.1;
  public final double ROLLER_ADJUST_FORWARD =  0.4;
  
  // -----------------------------------------------------------------------------------------------
  // Beam Break Definitions and Variables
  // -----------------------------------------------------------------------------------------------
  public static final boolean BEAM_IS_BROKEN     = true;
  public static final boolean BEAM_IS_NOT_BROKEN = false;
  
  public boolean m_desiredBeamBreakState;
}
