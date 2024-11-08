// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindex.Spindex;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.utils.Util;

public final class AutoRoutines {
  /** Example static factory for an autonomous command. */
  private final CommandSwerveDrivetrain swerve;

  private final PivotShooter pivotShooter;
  private final Intake intake;
  private final Shooter shooter;
  private final Spindex spindex;
  private final Turret turret;

  public Command boxAuto(AutoFactory factory) {
    final AutoLoop loop = factory.newLoop("box");

    final AutoTrajectory box = factory.trajectory("box", loop);

    loop.enabled().onTrue(swerve.resetOdometryCmd(box).andThen(box.cmd()));

    return loop.cmd();
  }

  public Command sourceLeave(AutoFactory factory) {
    final AutoLoop loop = factory.newLoop("Source Leave");

    final AutoTrajectory sourceLeave = factory.trajectory("Source-Leave", loop);
    loop.enabled()
        .onTrue(swerve.resetOdometryCmd(sourceLeave))
        .onTrue(AutoCommands.preload(pivotShooter, spindex, shooter, turret))
        .debounce(7)
        .onTrue(sourceLeave.cmd());
    return loop.cmd();
  }

  public Command ampLeave(AutoFactory factory) {
    final AutoLoop loop = factory.newLoop("Amp Leave");

    final AutoTrajectory ampLeave = factory.trajectory("Amp-Leave", loop);
    loop.enabled()
        .onTrue(swerve.resetOdometryCmd(ampLeave))
        .onTrue(AutoCommands.preload(pivotShooter, spindex, shooter, turret))
        .debounce(7)
        .onTrue(ampLeave.cmd());
    return loop.cmd();
  }

  public AutoRoutines(
      CommandSwerveDrivetrain swerve,
      PivotShooter pivotShooter,
      Intake intake,
      Shooter shooter,
      Spindex spindex,
      Turret turret) {
    this.swerve = swerve;
    this.pivotShooter = pivotShooter;
    this.intake = intake;
    this.shooter = shooter;
    this.spindex = spindex;
    this.turret = turret;
  }

  private static final class AutoCommands {
    private AutoCommands() {
      throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Command preload(
        PivotShooter pivotShooter, Spindex spindex, Shooter shooter, Turret turret) {
      return Commands.parallel(
          pivotShooter.setPosition(PivotShooterConstants.kSubWooferPreset),
          turret.setPosition(TurretConstants.kSubPreset),
          shooter.setVelocity(
              ShooterConstants.kShooterSpeakerRPS, ShooterConstants.kShooterFollowerSpeakerRPS),
          Commands.waitUntil(
                  () ->
                      Util.epsilonEquals(
                              shooter.getMainVelocity(), ShooterConstants.kShooterSpeakerRPS, 5)
                          && Util.epsilonEquals(
                              shooter.getFollowerVelocity(),
                              ShooterConstants.kShooterFollowerSpeakerRPS,
                              5))
              .andThen(spindex.feedNoteToShooter()));
    }
  }
}
