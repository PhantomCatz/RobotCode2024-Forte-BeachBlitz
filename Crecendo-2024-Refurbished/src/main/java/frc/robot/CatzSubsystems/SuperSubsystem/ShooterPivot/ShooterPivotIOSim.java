package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.CatzConstants;

public class ShooterPivotIOSim implements ShooterPivotIO {

    private final DCMotorSim elevationSimMotor;

    Mechanism2d shooterPivotVisualizer = new Mechanism2d(3, 3);
    MechanismRoot2d root = shooterPivotVisualizer.getRoot("PivotPoint", 0, 0);

    private final PIDController shooterPivotFeedback = new PIDController(2,0,0, CatzConstants.LOOP_TIME);

    
    public ShooterPivotIOSim() {

        elevationSimMotor = new DCMotorSim(DCMotor.getNeo550(1), 0, 0);
    }


    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        inputs.positionTicks = elevationSimMotor.getAngularPositionRotations();
    }

    @Override
    public void runPercentOutput(double percentOutput) {

    }

    @Override
    public void runSetpointTicks(double currentPosition, double setPointTicks) {
        elevationSimMotor.setInput(
            shooterPivotFeedback.calculate(currentPosition, setPointTicks)
        );
    }
}
