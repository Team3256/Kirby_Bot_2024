// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import edu.wpi.first.wpilibj2.command.Command;
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

  // private beambreak Beambreak = new beamreak(1)
  public Spindex(boolean disabled, SpindexIO spindexIO, BeamBreakIO beamBreakIO) {
    super(disabled);
    this.spindexIO = spindexIO;
    this.beamBreakIO = beamBreakIO;
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
    return this.run(() -> spindexIO.setSpindexVoltage(voltage)).finallyDo(() -> spindexIO.off());
  }

  public Command setSpindexVelocity(double velocity) {
    return this.run(() -> spindexIO.setSpindexVelocity(velocity)).finallyDo(() -> spindexIO.off());
  }

  public Command goToShooter() {
    return setSpindexVelocity(SpindexConstants.spindexMotorSpeedRPS)
        .until(() -> beamBreakIOAutoLogged.beamBroken)
        .andThen(this.off());
  }

  public Command off() {
    return this.run(() -> spindexIO.off());
  }
}
