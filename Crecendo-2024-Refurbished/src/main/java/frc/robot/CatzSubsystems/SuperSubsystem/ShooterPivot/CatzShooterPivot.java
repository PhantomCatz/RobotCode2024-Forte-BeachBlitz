// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utilities.Alert;
import frc.robot.Utilities.AutoAimingParametersUtil;
import frc.robot.Utilities.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

public class CatzShooterPivot {
  
  // Hardware IO Declaration
  private final ShooterPivotIO io;
  private final ShooterPivotIOInputsAutoLogged inputs = new ShooterPivotIOInputsAutoLogged();

  // Misc variables
  private ShooterPivotPositionType m_targetPosition;
  private double targetShooterPivotPosition = 0.0;
  private double manualPwr = 0.0;
  private boolean isCharacterizing = false;

  // Motor Console alerts
  private final Alert disconnectedAlertShooterPivot   = new Alert("Shooter Pivot motor disconnected!", Alert.AlertType.WARNING);



  //State machine
  @RequiredArgsConstructor
  public enum ShooterPivotPositionType {
    AUTO_AIM(()-> AutoAimingParametersUtil.getAutoAimSpeakerParemeters()
                                          .shooterPivotAngle()
                                          .getDegrees()), // TODO add auto aim parameters
    MANUAL(() -> 0.0),
    HOME(new LoggedTunableNumber("shooterPivot/tunnable/home", 202)),
    SUBWOOFER(new LoggedTunableNumber("shooterPivot/Tunnable/subwoofer", 10)),
    TEST(new LoggedTunableNumber("shooterPivot/Tunnable/TestingTicks", 7));

    private final DoubleSupplier motionType;
    private double getTargetMotionPosition() {
      return motionType.getAsDouble();
    }
  }


  public CatzShooterPivot() {
     if(ShooterPivotConstants.isShooterPivotDisabled) {
      io = new ShooterPivotIONull();
      System.out.println("Shooter Pivot Unconfigured");
    } else {
      switch (CatzConstants.hardwareMode) {
        case REAL:
          io = new ShooterPivotIOReal();
          System.out.println("Shooter Pivot Configured for Real");
        break;
        case REPLAY:
          io = new ShooterPivotIOReal() {};
          System.out.println("Shooter Pivot Configured for Replayed simulation");
        break;
        case SIM:
          io = new ShooterPivotIOSim();
          System.out.println("Shooter Pivot Configured for WPILIB simulation");
        break;
        default:
          io = null;
          System.out.println("Shooter Pivot Unconfigured");
        break;
      }
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("inputs/ShooterPivot", inputs);
    
    // Update Controllers when User Specifies
    LoggedTunableNumber.ifChanged(
      hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

    // Set Alerts
    disconnectedAlertShooterPivot.set(!inputs.isElevationMotorConnected);
    // Run Setpoint Control
    if(DriverStation.isDisabled() || m_targetPosition == null) {
      io.stop();
      m_targetPosition = null;
    } else if(m_targetPosition == ShooterPivotPositionType.MANUAL) {
      io.runPercentOutput(manualPwr);
    } else {
      io.runSetpointTicks(getPositionTicks(), m_targetPosition.getTargetMotionPosition());
    }

  }

  //-----------------------------------------------------------------------------------------
  //
  //    Misc Methods
  //
  //-----------------------------------------------------------------------------------------
  public double getPositionTicks() {
    return inputs.positionTicks;
  }

  public void setNeutralMode() {
    io.stop();
  }

  public void setPercentOutput(double percentOutput) {
    m_targetPosition = ShooterPivotPositionType.MANUAL;
    manualPwr = percentOutput;
  }


  //-----------------------------------------------------------------------------------------
  //
  //    State Setting method
  //
  //-----------------------------------------------------------------------------------------
  public void setTargetMotionMethod(ShooterPivotPositionType type) {
    m_targetPosition = type;
  }
}
