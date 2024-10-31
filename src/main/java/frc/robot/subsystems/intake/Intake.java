// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.BeamBreakIO;
import frc.robot.subsystems.BeamBreakIOInputsAutoLogged;
import frc.robot.utils.DisableSubsystem;
import frc.robot.utils.generics.SingleMotorSubsystemIO;
import frc.robot.utils.generics.SingleMotorSubsystemInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake
    extends DisableSubsystem { // note for me later - this has also controls the redirect roller
  private final SingleMotorSubsystemIO intakeIO;
  private final SingleMotorSubsystemInputsAutoLogged intakeIOAutoLogged = new SingleMotorSubsystemInputsAutoLogged();
  private final SysIdRoutine intake_sysIdRoutine;

  private final Trigger debouncedBeamBreak = new Trigger(this::isBeamBroken).debounce(0.1);

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakIOAutoLogged = new BeamBreakIOInputsAutoLogged();

  public Intake(boolean disabled, SingleMotorSubsystemIO intakeIO, BeamBreakIO beamBreakIO) {
    super(disabled);

    this.intakeIO = intakeIO;
    this.beamBreakIO = beamBreakIO;
    intake_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.2).per(Seconds.of(1)), // Use default ramp rate (1 V/s)
            Volts.of(6), // Reduce dynamic step voltage to 4 to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> intakeIO
                .getMotor()
                .setControl(intakeIO.getVoltageRequest().withOutput(volts.in(Volts))),
            null,
            this));
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeIO.updateInputs(intakeIOAutoLogged);
    beamBreakIO.updateInputs(beamBreakIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), intakeIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), beamBreakIOAutoLogged);
  }

  public Command setVoltage(double voltage) {
    return this.run(() -> intakeIO.setIntakeVoltage(voltage)).finallyDo(intakeIO::off);
  }

  public Command setVelocity(double velocity, double passthroughVelocity) {
    return this.run(() -> intakeIO.setIntakeVelocity(velocity)).finallyDo(intakeIO::off);
  }

  public Command off() {
    return this.runOnce(intakeIO::off);
  }

  public Command intakeIn() {
    return this.run(() -> intakeIO.setVoltage(IntakeConstants.kMotorVoltage))
        .until(debouncedBeamBreak)
        .finallyDo(intakeIO::off);
  }

  public Command redirectToAmp() {
    return this.run(() -> intakeIO.setVoltage(IntakeConstants.kIntakeRedirectVoltage))
        .finallyDo(intakeIO::off);
  }

  public Command intakeSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intake_sysIdRoutine.quasistatic(direction);
  }

  public Command intakeSysIdDynamic(SysIdRoutine.Direction direction) {
    return intake_sysIdRoutine.dynamic(direction);
  }

  public boolean isBeamBroken() {
    return beamBreakIOAutoLogged.beamBroken;
  }
}
