// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevatorrollers;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.BeamBreakIO;
import frc.robot.subsystems.BeamBreakIOInputsAutoLogged;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Roller extends DisableSubsystem {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerIOAutoLogged = new RollerIOInputsAutoLogged();

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakIOAutoLogged =
      new BeamBreakIOInputsAutoLogged();

  private final SysIdRoutine roller_sysIdRoutine;

  private final Trigger debouncedBeamBreak = new Trigger(this::isBeamBroken).debounce(0.1);
  ;

  public Roller(boolean disabled, RollerIO rollerIO, BeamBreakIO beamBreakIO) {
    super(disabled);

    this.rollerIO = rollerIO;
    this.beamBreakIO = beamBreakIO;
    roller_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1)), // Use default ramp rate (1 V/s)
                Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (volts) ->
                    rollerIO
                        .getRollerMotor()
                        .setControl(rollerIO.getRollerVoltageRequest().withOutput(volts.in(Volts))),
                null,
                this));
  }

  @Override
  public void periodic() {
    super.periodic();
    rollerIO.updateInputs(rollerIOAutoLogged);
    beamBreakIO.updateInputs(beamBreakIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), rollerIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), beamBreakIOAutoLogged);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> rollerIO.setRollerVoltage(voltage)).finallyDo(rollerIO::off);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return this.run(() -> rollerIO.setRollerVelocity(velocity)).finallyDo(rollerIO::off);
  }

  public Command off() {
    return this.runOnce(rollerIO::off);
  }

  public Command outtake() {
    return this.run(() -> rollerIO.setRollerVoltage(RollerConstants.kRollerRollerVoltage))
        .until(debouncedBeamBreak)
        .andThen(this.off());
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return roller_sysIdRoutine.quasistatic(direction);
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return roller_sysIdRoutine.dynamic(direction);
  }

  public boolean isBeamBroken() {
    return beamBreakIOAutoLogged.beamBroken;
  }
}
