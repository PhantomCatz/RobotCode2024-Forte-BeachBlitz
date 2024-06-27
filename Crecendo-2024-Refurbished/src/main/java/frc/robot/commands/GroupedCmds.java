package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class GroupedCmds {
    

    public Command IntakeToShooterSequence() {
        return new SequentialCommandGroup(null);
    }
}
