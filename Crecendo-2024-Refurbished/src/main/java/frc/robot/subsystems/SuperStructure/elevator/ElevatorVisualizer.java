// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.elevator;

import static frc.robot.subsystems.SuperStructure.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ElevatorVisualizer {
  private final Mechanism2d mechanism;
  private final MechanismLigament2d arm;
  private final String key;

  public ElevatorVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new Mechanism2d(3.0, 3.0, new Color8Bit(Color.kWhite));
    MechanismRoot2d root = mechanism.getRoot("pivot", 1.0, 0.4);
    arm = new MechanismLigament2d("arm", elevatorLength, 20.0, 6, new Color8Bit(color));
    root.append(arm);
  }

  /** Update arm visualizer with current arm angle */
  public void update(double angleRads) {
    // Log Mechanism2d
    arm.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Elevator/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(elevatorOrigin.getX(), 0.0, elevatorOrigin.getY(), new Rotation3d(0.0, -angleRads, 0.0));
    Logger.recordOutput("Elvator/Mechanism3d/" + key, pivot);
  }


}
