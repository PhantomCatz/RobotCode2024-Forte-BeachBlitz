// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakeRollers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO
{
    private TalonFX              rollerMtr;
    private TalonFXConfiguration talonConfigsRoller = new TalonFXConfiguration();

    private StatusCode rollerInitializationStatus = StatusCode.StatusCodeNotInitialized;

    public DigitalInput adjustBeamBreak = new DigitalInput(4);
    public DigitalInput loadBeamBreak   = new DigitalInput(5);
    
    //Motor Instantiation
    public IntakeIOReal()
    {
        // -----------------------------------------------------------------------------------------------
        // 
        // Rollers
        // 
        // -----------------------------------------------------------------------------------------------
        rollerMtr = new TalonFX(IntakeConstants.ROLLER_MTR_ID);
        rollerInitializationStatus = rollerMtr.getConfigurator().apply(new TalonFXConfiguration());  //reset to factory defaults

        talonConfigsRoller.CurrentLimits = new CurrentLimitsConfigs();

        talonConfigsRoller.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.INTAKE_ROLLER_ENABLE_CURRENT_LIMIT;
        talonConfigsRoller.CurrentLimits.StatorCurrentLimit       = IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT_AMPS;

        talonConfigsRoller.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.INTAKE_ROLLER_ENABLE_CURRENT_LIMIT;
        talonConfigsRoller.CurrentLimits.SupplyCurrentLimit       = IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT_AMPS;
        talonConfigsRoller.CurrentLimits.SupplyCurrentThreshold   = IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigsRoller.CurrentLimits.SupplyTimeThreshold      = IntakeConstants.INTAKE_ROLLER_CURRENT_LIMIT_TIMEOUT_SECONDS;

        talonConfigsRoller.MotorOutput.NeutralMode                = NeutralModeValue.Brake;
        
        // -----------------------------------------------------------------------------------------------
        // Check if Roller Motor was initialized correctly
        // -----------------------------------------------------------------------------------------------
        rollerInitializationStatus = rollerMtr.getConfigurator().apply(talonConfigsRoller);
        if(!rollerInitializationStatus.isOK()) {
            System.out.println("Intake Roller Mtr (" + IntakeConstants.ROLLER_MTR_ID + "): " + 
                                                       rollerInitializationStatus.getName() + ":" +
                                                       rollerInitializationStatus.getDescription());
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        //true if beambreak is broken \/ \/
        inputs.AdjustBeamBrkState =     !adjustBeamBreak.get(); 
        inputs.LoadBeamBrkState   =     !loadBeamBreak.get();
    }

    // -----------------------------------------------------------------------------------------------
    // 
    // Roller Methods
    // 
    // -----------------------------------------------------------------------------------------------
    @Override
    public void setRollerPercentOutput (double percentOutput)
    {
        rollerMtr.set(percentOutput);
    }
}
