// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

  private final EncoderIO encoderIO1;
  private final EncoderIOInputsAutoLogged encoderIOInputs1 = new EncoderIOInputsAutoLogged();

  private final EncoderIO encoderIO2;
  private final EncoderIOInputsAutoLogged encoderIOInputs2 = new EncoderIOInputsAutoLogged();

  public Turret(TurretIO turretIO, EncoderIO encoderIO1, EncoderIO encoderIO2) {
    this.turretIO = turretIO;
    this.encoderIO1 = encoderIO1;
    this.encoderIO2 = encoderIO2;
    this.setDefaultCommand(reset());
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretIOInputs);
    Logger.processInputs(this.getClass().getSimpleName(), turretIOInputs);
    encoderIO1.updateInputs(encoderIOInputs1);
    Logger.processInputs(this.getClass().getSimpleName() + "/encoder1", encoderIOInputs1);
    encoderIO2.updateInputs(encoderIOInputs2);
    Logger.processInputs(this.getClass().getSimpleName() + "/encoder2", encoderIOInputs2);

  }

  public static Rotation2d getTurretPosition(Rotation2d cancoder1, Rotation2d cancoder2) {
    Rotation2d diff = cancoder2.minus(cancoder1);
    double drivingRotRaw = diff.getDegrees() / TurretConstants.differenceDegrees;
    double expectedG1 = (drivingRotRaw * TurretConstants.ratio1) % 1;
    double g1Diff = expectedG1 - (cancoder1.getRotations() % 1);
    if (g1Diff > 0.5) {
      g1Diff -= 1;
    } else if (g1Diff < -0.5) {
      g1Diff += 1;
    }
    double drivingRem = g1Diff / TurretConstants.ratio1;
    return Rotation2d.fromRotations(drivingRotRaw - drivingRem);
  }

  public Command setPositionRelativeToSwerve(Rotation2d position, Rotation2d swerveAngle) {
    return new StartEndCommand(
        () ->
            turretIO.setPosition(
                Units.degreesToRotations(position.getDegrees() - swerveAngle.getDegrees())),
        () -> {},
        this);
  }

  public Command setPosition(Rotation2d position) {
    return new StartEndCommand(
        () -> turretIO.setPosition(Units.degreesToRotations(position.getDegrees())),
        () -> {},
        this);
  }

  public Command zero() {
    return new StartEndCommand(() -> turretIO.zero(), () -> {}, this);
  }

  public Command reset() {
    return new StartEndCommand(
        () -> {
          if (turretIOInputs.turretMotorPosition > TurretConstants.kForwardLimit) {
            turretIO.setPosition(0);
          } else if (turretIOInputs.turretMotorPosition < TurretConstants.kReverseLimit) {
            turretIO.setPosition(0);
          }
        },
        () -> {},
        this);
  }
}
