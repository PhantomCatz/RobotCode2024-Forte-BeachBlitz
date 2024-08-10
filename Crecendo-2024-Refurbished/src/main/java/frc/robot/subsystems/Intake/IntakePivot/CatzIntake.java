// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CatzConstants;

public class CatzIntake extends SubsystemBase {
  // -----------------------------------------------------------------------------------------------
  // Intake IO Block
  // -----------------------------------------------------------------------------------------------
  private final IntakeIO io;
  
  // private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged(); //TODO since related to io.update, delete?

  // -----------------------------------------------------------------------------------------------
  // 
  // Intake Rollers
  // 
  // -----------------------------------------------------------------------------------------------
  
  // -----------------------------------------------------------------------------------------------
  // Roller States
  // -----------------------------------------------------------------------------------------------
  public static enum IntakeRollerState {
    ROLLERS_IN_SOURCE,
    ROLLERS_IN_GROUND,
    BEAM_BREAK_CHECK,
    NOTE_ADJUST,
    ROLLERS_OUT_EJECT,
    ROLLERS_OUT_SHOOTER_HANDOFF,
    ROLLERS_OFF, 
  }

  public IntakeRollerState m_currentRollerState;

  // -----------------------------------------------------------------------------------------------
  // 
  // CatzIntake
  // 
  // -----------------------------------------------------------------------------------------------
  public CatzIntake()
  {
    switch (CatzConstants.currentMode)
    {
      case REAL:
        io = new IntakeIOReal();
        System.out.println("Intake Configured for Real");
        break;

      case REPLAY:
        io = new IntakeIOReal() {};
        System.out.println("Intake Configured for Replayed simulation");
        break;

      case SIM:
      default:
        io = null;
        System.out.println("Intake Unconfigured");
        break;
    }
  }

  @Override
  public void periodic() {
    // io.updateInputs(inputs);
    // Logger.processInputs("intake/inputs", inputs);
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Roller Methods
  // 
  // -------------------------------------------------------------------------------------
  public void setRollerIn ()
  {
    io.setRollerPercentOutput(IntakeConstants.ROLLERS_MTR_PWR_IN_GROUND);
  }

  public void setRollerOut()
  {
    io.setRollerPercentOutput(IntakeConstants.ROLLERS_MTR_PWR_OUT_EJECT);
  }

  public void setRollerOff ()
  {
    io.setRollerPercentOutput(0.0);
  }

  // -------------------------------------------------------------------------------------
  // 
  // Intake Roller Commands
  // 
  // -------------------------------------------------------------------------------------
  public Command cmdRollerIn ()
  {
    return runOnce(() -> setRollerIn());
  }

  public Command cmdRollerOut()
  {
    return runOnce(() -> setRollerOut());
  }

  public Command cmdRollerOff()
  {
    return runOnce(() -> setRollerOff());
  }

  public Command cmdStopRollersAfterTimeOut()
  {
    return new ParallelDeadlineGroup(new WaitCommand(2), cmdRollerIn());
  }

  // public Command cmdStopRollersAfterTimeOut()
  // {
  //   return new SequentialCommandGroup(
  //     cmdRollerIn(),
  //     new WaitCommand(2),
  //     cmdRollerOff()
  //   );
  // }
}
