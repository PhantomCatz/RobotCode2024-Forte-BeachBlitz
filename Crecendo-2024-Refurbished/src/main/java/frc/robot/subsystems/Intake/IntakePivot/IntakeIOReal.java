// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO
{
    private TalonFX pivotMtr;
    private TalonFX rollerMtr;

    private StatusCode pivotInitializationStatus = StatusCode.StatusCodeNotInitialized;
    private StatusCode rollerInitializationStatus = StatusCode.StatusCodeNotInitialized;

    private TalonFXConfiguration talonConfigsPivot = new TalonFXConfiguration();
    private TalonFXConfiguration talonConfigsRoller = new TalonFXConfiguration();

    public DigitalInput adjustBeamBreak = new DigitalInput(4);
    public DigitalInput loadBeamBreak = new DigitalInput(5);
    
    //Motor Instantiation
    public IntakeIOReal()
    {
        // -----------------------------------------------------------------------------------------------
        // Pivot
        // -----------------------------------------------------------------------------------------------
        pivotMtr = new TalonFX(IntakeConstants.PIVOT_MTR_ID);
        pivotMtr.getConfigurator().apply(new TalonFXConfiguration());

        // -----------------------------------------------------------------------------------------------
        // Set Motion Magic Settings
        // -----------------------------------------------------------------------------------------------
        talonConfigsPivot.MotionMagic.MotionMagicCruiseVelocity = 40;
        talonConfigsPivot.MotionMagic.MotionMagicAcceleration = 160;
        talonConfigsPivot.MotionMagic.MotionMagicJerk = 800;

        talonConfigsPivot.Slot0.kP = CatzIntake.PIVOT_PID_kP;
        talonConfigsPivot.Slot0.kI = CatzIntake.PIVOT_PID_kI;
        talonConfigsPivot.Slot0.kD = CatzIntake.PIVOT_PID_kD;

        // -----------------------------------------------------------------------------------------------
        // Set Current Limit
        // -----------------------------------------------------------------------------------------------
        talonConfigsPivot.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigsPivot.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.KRAKEN_ENABLE_CURRENT_LIMIT;
        talonConfigsPivot.CurrentLimits.StatorCurrentLimit = IntakeConstants.KRAKEN_CURRENT_LIMIT_AMPS;

        talonConfigsPivot.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.KRAKEN_ENABLE_CURRENT_LIMIT;
        talonConfigsPivot.CurrentLimits.SupplyCurrentLimit = IntakeConstants.KRAKEN_CURRENT_LIMIT_AMPS;
        talonConfigsPivot.CurrentLimits.SupplyCurrentThreshold = IntakeConstants.KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigsPivot.CurrentLimits.SupplyCurrentThreshold = IntakeConstants.KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS;

        talonConfigsPivot.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotMtr.setPosition(IntakeConstants.INTAKE_PIVOT_MTR_POS_OFFSET_IN_REV);

        // -----------------------------------------------------------------------------------------------
        // Check if Wrist Motor was initialized correctly
        // -----------------------------------------------------------------------------------------------
        pivotInitializationStatus = pivotMtr.getConfigurator().apply(talonConfigsPivot);
        if (!pivotInitializationStatus.isOK())
        {
            System.out.println("Failed to Configure Pivot Mtr Controller CAN ID" + IntakeConstants.PIVOT_MTR_ID);
        }

        // -----------------------------------------------------------------------------------------------
        // 
        // Rollers
        // 
        // -----------------------------------------------------------------------------------------------
        rollerMtr = new TalonFX(IntakeConstants.ROLLER_MTR_ID);
        rollerMtr.getConfigurator().apply(new TalonFXConfiguration());  //reset to factory defaults

        talonConfigsRoller = talonConfigsPivot;
        talonConfigsRoller.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        //check if roller motor is initialized correctly
        rollerInitializationStatus = rollerMtr.getConfigurator().apply(talonConfigsRoller);
        if(!rollerInitializationStatus.isOK()) {
            System.out.println("Failed to Configure Roller Mtr Controller CAN ID" + IntakeConstants.ROLLER_MTR_ID);
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotMtrRev =            pivotMtr.getPosition().getValue();
        inputs.closedLoopPivotMtr =     pivotMtr.getClosedLoopError().getValue();

        //true if beambreak is broken \/ \/
        inputs.AdjustBeamBrkState =     !adjustBeamBreak.get(); 
        inputs.LoadBeamBrkState =       !loadBeamBreak.get();
    }

    @Override
    public void resetPivotEncPos(double defaultEncPos) {
       pivotMtr.setPosition(defaultEncPos);
    }

    @Override
    public void setIntakePivotPercentOutput(double percentOutput) {
       pivotMtr.set(-percentOutput);
    }

    @Override
    public void setIntakePivotVoltage(double volts) {
        pivotMtr.setControl(new VoltageOut(volts));
    }

    @Override
    public void setIntakePivotPostionRev(double encOutput, double ffVoltage) {
        pivotMtr.setControl(new MotionMagicVoltage(encOutput, 
                                                     true, 
                                                     ffVoltage, 
                                                     0, 
                                                     false, 
                                                     false, 
                                                     false));
    }

    
}
