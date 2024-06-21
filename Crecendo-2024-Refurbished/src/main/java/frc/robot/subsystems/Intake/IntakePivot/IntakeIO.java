// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.IntakePivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO
{
    @AutoLog
    public static class IntakeIOInputs
    {
        public double pivotMtrRev;
        public double closedLoopPivotMtr;

        public boolean AdjustBeamBrkState;
        public boolean LoadBeamBrkState;
    }
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void rollerEnable(boolean enable) {}

    public default void setRollerPercentOutput(double speed) {}

    public default void rollerIn() {}

    public default void rollerOut() {}

    public default void rollerDisable() {}

    public default void setSquishyMode(boolean enable) {} 

    public default void resetPivotEncPos(double defaultEncoderPosition) {}

    public default void setIntakePivotVoltage(double volts) {}

    public default void setIntakePivotPercentOutput(double percentOutput) {}

    public default void setIntakePivotPostionRev(double pivotEncOuput, double ffVoltage) {}

}