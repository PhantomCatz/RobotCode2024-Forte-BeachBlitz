// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO
{
    @AutoLog
    public static class IntakeIOInputs
    {
        public boolean AdjustBeamBrkState;
        public boolean LoadBeamBrkState;
    }
    
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRollerPercentOutput(double speed) {}

    public default void rollerIn() {}

    public default void rollerOut() {}

    public default void rollerDisable() {}
}