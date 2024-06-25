// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SuperstructureCommandLogger {
    public static SUPERSTATE_COMMAND previousSuperStateCommand;
    public static enum SUPERSTATE_COMMAND {
      STOW,
      AUTO_AIM,
      SCORE_AMP,
      INTAKE_GROUND,
      INTAKE_SOURCE
    }

}
