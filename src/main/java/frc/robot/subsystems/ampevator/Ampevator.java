// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Ampevator extends DisableSubsystem {
  private final AmpevatorIO ampevatorIO;
  private final AmpevatorIOInputsAutoLogged ampevatorIOAutoLogged =
      new AmpevatorIOInputsAutoLogged();

  public Ampevator(boolean disabled, AmpevatorIO ampevatorIO) {
    super(disabled);
    this.ampevatorIO = ampevatorIO;
  }

  @Override
  public void periodic() {
    super.periodic();
    ampevatorIO.updateInputs(ampevatorIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), ampevatorIOAutoLogged);
  }

  public Command setPosition(double position) {
    return this.run(() -> ampevatorIO.setPosition(position));
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> ampevatorIO.setVoltage(voltage)).finallyDo(ampevatorIO::off);
  }

  public Command zero() {
    return this.run(ampevatorIO::zero);
  }

  public Command off() {
    return this.run(ampevatorIO::off);
  }

  public Command extendAmpevator() {
    return this.run(() -> ampevatorIO.setPosition(AmpevatorConstants.kAmpevatorUpPosition));
  }

  public Command retractAmpevator() {
    return this.run(() -> ampevatorIO.setPosition(AmpevatorConstants.kAmpevatorDownPosition));
  }
}
