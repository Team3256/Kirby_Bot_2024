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
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public final class AutoRoutines {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private final CommandSwerveDrivetrain swerve;

  public Command boxAuto(AutoFactory factory) {
    final AutoLoop loop = factory.newLoop("Box");

    final AutoTrajectory box = factory.trajectory("box", loop);

    loop.enabled().onTrue(Commands.runOnce(() -> swerve.resetOdometry(box)).andThen(box.cmd()));

    return loop.cmd();
  }

  public AutoRoutines(CommandSwerveDrivetrain swerve) {
    this.swerve = swerve;
  }

  private static final class AutoCommands {
    private AutoCommands() {
      throw new UnsupportedOperationException("This is a utility class!");
    }
  }
}
