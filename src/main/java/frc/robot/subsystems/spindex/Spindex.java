// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BeamBreakIO;
import frc.robot.subsystems.BeamBreakIOInputsAutoLogged;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

public class Spindex extends DisableSubsystem {

  private final SpindexIO spindexIO;
  private final SpindexIOInputsAutoLogged spindexIOAutoLogged = new SpindexIOInputsAutoLogged();

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakIOAutoLogged =
      new BeamBreakIOInputsAutoLogged();

  private final ShooterFeederIO shooterFeederIO;
  private final ShooterFeederIOInputsAutoLogged shooterFeederIOAutoLogged =
      new ShooterFeederIOInputsAutoLogged();

  public final Trigger debouncedBeamBreak = new Trigger(() -> beamBreakIOAutoLogged.beamBroken);

  // private beambreak Beambreak = new beamreak(1)
  public Spindex(
      boolean disabled,
      SpindexIO spindexIO,
      ShooterFeederIO shooterFeeder,
      BeamBreakIO beamBreakIO) {
    super(disabled);
    this.spindexIO = spindexIO;
    this.beamBreakIO = beamBreakIO;
    this.shooterFeederIO = shooterFeeder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    spindexIO.updateInputs(spindexIOAutoLogged);
    beamBreakIO.updateInputs(beamBreakIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), spindexIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), beamBreakIOAutoLogged);
  }

  public Command setSpindexVoltage(double voltage) {
    return this.run(() -> spindexIO.setSpindexVoltage(voltage)).finallyDo(spindexIO::off);
  }

  public Command setSpindexVelocity(double velocity) {
    return this.run(() -> spindexIO.setSpindexVelocity(velocity)).finallyDo(spindexIO::off);
  }

  public Command setShooterFeederVoltage(double voltage) {
    return this.run(() -> shooterFeederIO.setFeederVoltage(voltage))
        .finallyDo(shooterFeederIO::off);
  }

  public Command setShooterFeederVelocity(double velocity) {
    return this.run(() -> shooterFeederIO.setFeederVelocity(velocity))
        .finallyDo(shooterFeederIO::off);
  }

  public Command setVoltage(double spindexVoltage, double shooterFeederVoltage) {
    return this.run(
        () -> {
          spindexIO.setSpindexVoltage(spindexVoltage);
          shooterFeederIO.setFeederVoltage(shooterFeederVoltage);
        });
  }

  public Command goToShooter() {
    return setVoltage(SpindexConstants.spindexMotorVoltage, SpindexConstants.shooterFeederVoltage)
        .until(() -> beamBreakIOAutoLogged.beamBroken)
        .finallyDo(
            () -> {
              spindexIO.off();
              shooterFeederIO.off();
            });
  }

  public Command feedNoteToShooter() {
    return setVoltage(SpindexConstants.spindexMotorVoltage, SpindexConstants.shooterFeederVoltage)
//        .until(() -> !beamBreakIOAutoLogged.beamBroken)
            .finallyDo(
                    () -> {
                      spindexIO.off();
                      shooterFeederIO.off();
                    });
  }

  public Command goToAmpevator() {
    return setSpindexVoltage(-SpindexConstants.spindexMotorVoltage)
        .until(() -> beamBreakIOAutoLogged.beamBroken)
        .finallyDo(spindexIO::off);
  }

  public Command off() {
    return this.run(
        () -> {
          spindexIO.off();
          shooterFeederIO.off();
        });
  }
}
