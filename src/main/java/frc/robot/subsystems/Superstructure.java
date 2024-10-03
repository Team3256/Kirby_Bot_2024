// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ampevator.Ampevator;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spindex.Spindex;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static enum StructureState {
    IDLE,
    HOMED,
    PREINTAKE,
    INTAKE,
    PRECLIMB,
    CLIMB,
    PRESUB,
    PREPODIUM,
    PREFEED,
    SHOOT,
    PREAMP,
    OUTTAKE,
  }

  private final Ampevator ampevator;
  private final Spindex spindex;
  private final Climb climb;
  private final Intake intake;
  private final PivotShooter pivotShooter;
  private final Shooter shooter;
  private final Vision vision;

  private StructureState state = StructureState.IDLE;
  private StructureState prevState = StructureState.IDLE;

  private Map<StructureState, Trigger> stateTriggers = new HashMap<StructureState, Trigger>();

  private Timer stateTimer = new Timer();

  public Superstructure(
      Ampevator ampevator,
      Climb climb,
      Intake intake,
      Spindex spindex,
      PivotShooter pivotShooter,
      Shooter shooter,
      Vision vision) {
    this.ampevator = ampevator;
    this.climb = climb;
    this.intake = intake;
    this.spindex = spindex;
    this.pivotShooter = pivotShooter;
    this.shooter = shooter;
    this.vision = vision;

    stateTimer.start();

    for (StructureState state : StructureState.values()) {
      stateTriggers.put(state, new Trigger(() -> this.state == state));
    }

    configStateTransitions();
  }

  public void configStateTransitions() {
    stateTriggers
        .get(StructureState.IDLE)
        .onTrue(ampevator.off())
        .onTrue(climb.off())
        .onTrue(intake.off())
        .onTrue(spindex.off())
        .onTrue(pivotShooter.off())
        .onTrue(shooter.off());
  }

  // call manually
  public void periodic() {
    Logger.recordOutput(this.getClass().getSimpleName() + "/State", this.state.toString());
    Logger.recordOutput(this.getClass().getSimpleName() + "/PrevState", this.prevState.toString());
    Logger.recordOutput(this.getClass().getSimpleName() + "/StateTime", this.stateTimer.get());
  }

  public Command setState(StructureState state) {
    return Commands.runOnce(
        () -> {
          this.prevState = this.state;
          this.state = state;
          this.stateTimer.restart();
        });
  }
}