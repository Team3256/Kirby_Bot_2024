// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class SimMechs {

  public static final Mechanism2d mech = new Mechanism2d(5, 5);

  private static final MechanismRoot2d ampevator = mech.getRoot("ampevator", 1, 0);

  private static final MechanismRoot2d pivot = mech.getRoot("pivot", 4, 0);

  private static final MechanismLigament2d m_ampevator =
      ampevator.append(new MechanismLigament2d("ampevator", 2, 90));
  private static final MechanismLigament2d m_ampevatorRollers =
      m_ampevator.append(new MechanismLigament2d("rollers", 1, 0));

  private static final Mechanism2d shooter = new Mechanism2d(3.0, 3.0);

  private static final MechanismRoot2d pivotShooterRoot = shooter.getRoot("Pivot Shooter", 0.1, 0.1);
  private static final MechanismLigament2d pivotShooterViz =
      pivotShooterRoot.append(
          new MechanismLigament2d(
              "Pivot Shooter", 0.5, 0.0, 5.0, new Color8Bit(Color.kGreen)));
  private static final MechanismRoot2d leftShooterAxle =
      shooter.getRoot("Left Shooter Axle", 1.0, 2.0);
  private static final MechanismLigament2d leftShooterViz =
      leftShooterAxle.append(
          new MechanismLigament2d(
              "Left Shooter Flywheel", .5, 0.0, 5.0, new Color8Bit(Color.kYellow)));
  private static final MechanismRoot2d rightShooterAxle =
      shooter.getRoot("Right Shooter Axle", 2.0, 2.0);
  private static final MechanismLigament2d rightShooterViz =
      rightShooterAxle.append(
          new MechanismLigament2d(
              "Right Shooter Flywheel", 0.5, 0.0, 5.0, new Color8Bit(Color.kYellow)));

  public static void updateAmpevator(double position) {
    m_ampevator.setLength(position);
  }

  public static void addToShooterFlywheelAngle(double leftAngle, double rightAngle) {
    leftShooterViz.setAngle(leftShooterViz.getAngle() + leftAngle);
    rightShooterViz.setAngle(rightShooterViz.getAngle() + rightAngle);
  }

  public static void updatePivotShooter(Rotation2d angle) {
    pivotShooterViz.setAngle(angle.getDegrees());
  }

  public static void init() {
    SmartDashboard.putData("Ampevator", mech);
    SmartDashboard.putData("Shooter", shooter);
  }
}
