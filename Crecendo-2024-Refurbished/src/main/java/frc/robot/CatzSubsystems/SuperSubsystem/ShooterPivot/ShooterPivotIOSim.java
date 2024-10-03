package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.CatzConstants;

public class ShooterPivotIOSim implements ShooterPivotIO {

    private final DCMotorSim elevationSimMotor;

        private final PIDController shooterPivotFeedback = new PIDController(gains.kP(), gains.kI(), gains.kD(), CatzConstants.LOOP_TIME);

    
    public ShooterPivotIOSim() {

        elevationSimMotor = new DCMotorSim(DCMotor.getNeo550(1), 0, 0);
    }


    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {

    }
}
