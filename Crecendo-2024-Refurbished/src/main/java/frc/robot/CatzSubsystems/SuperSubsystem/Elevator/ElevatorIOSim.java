// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CatzSubsystems.SuperSubsystem.Elevator;

import static frc.robot.CatzSubsystems.SuperSubsystem.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.CatzConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
  private static final double autoStartAngle = Units.degreesToRadians(80.0);

  private final ElevatorSim sim = 
    new ElevatorSim(gains.kV(), gains.kA(), DCMotor.getKrakenX60(2), 0.0, 2, true, 0.0); //TODO fix min and max height


  private final PIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  public ElevatorIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
    setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(CatzConstants.LOOP_TIME);

    inputs.leaderPositionRotations = sim.getPositionMeters();
    inputs.velocityRps = sim.getVelocityMetersPerSecond();// TODO fix
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelcius = 0.0;

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void runSetpoint(double setpointRotations, double feedforward) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    runVolts(controller.calculate(sim.getPositionMeters(), setpointRotations) + feedforward); //TODO fix conversions
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controller.setPID(p, i, d);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

  private void setPosition(double position) { //TODO see if method is accuatley needed for elevator

  }
}
