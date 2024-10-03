// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;

// Singleton for simulation visualization
public enum SimViz implements Subsystem {
  INSTANCE;

  private static final Mechanism2d mech = new Mechanism2d(3.0, 3.0);
  private static final MechanismRoot2d leftShooterAxle =
      mech.getRoot("Left Shooter Axle", 1.0, 2.0);
  private static final MechanismLigament2d leftShooterViz =
      leftShooterAxle.append(
          new MechanismLigament2d(
              "Left Shooter Flywheel", .5, 0.0, 5.0, new Color8Bit(Color.kYellow)));
  private static final MechanismRoot2d rightShooterAxle =
      mech.getRoot("Right Shooter Axle", 2.0, 2.0);
  private static final MechanismLigament2d rightShooterViz =
      rightShooterAxle.append(
          new MechanismLigament2d(
              "Right Shooter Flywheel", 0.5, 0.0, 5.0, new Color8Bit(Color.kYellow)));

  private SimViz() {
    System.out.println("Here");
  }

  public static void init() {
    SmartDashboard.putData("/SimViz", mech);
  }

  public static SimViz getInstance() {
    return INSTANCE;
  }

  // angle is in degrees
  public void addToShooterFlywheelAngle(double leftAngle, double rightAngle) {
    leftShooterViz.setAngle(leftShooterViz.getAngle() + leftAngle);
    rightShooterViz.setAngle(rightShooterViz.getAngle() + rightAngle);
  }
}
