// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.DisableSubsystem;
import org.littletonrobotics.junction.Logger;

public class Turret extends DisableSubsystem {

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged turretIOInputs = new TurretIOInputsAutoLogged();

  private final EncoderIO encoderIO1;
  private final EncoderIOInputsAutoLogged encoderIOInputs1 = new EncoderIOInputsAutoLogged();

  private final EncoderIO encoderIO2;
  private final EncoderIOInputsAutoLogged encoderIOInputs2 = new EncoderIOInputsAutoLogged();

  public Turret(boolean disabled, TurretIO turretIO, EncoderIO encoderIO1, EncoderIO encoderIO2) {
    super(disabled);
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
    Logger.recordOutput(
        this.getClass().getSimpleName() + "/CRTposition",
        getTurretPosition(
            Rotation2d.fromDegrees(encoderIOInputs1.encoderPositionDegrees),
            Rotation2d.fromDegrees(encoderIOInputs2.encoderPositionDegrees)));
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
    return this.runOnce(
        () ->
            turretIO.setPosition(
                Units.degreesToRotations(position.getDegrees() - swerveAngle.getDegrees())));
  }

  public Command setPosition(Rotation2d position) {
    return this.runOnce(
        () ->
            turretIO.setPosition(
                Units.degreesToRotations(position.getDegrees()) / TurretConstants.gearRatio));
  }

  public Command zero() {
    return this.runOnce(() -> turretIO.zero());
  }

  public Command lockToSpeakerTag(Vision vision) {
    return new PIDCommand(
        new PIDController(
            TurretConstants.followTagP, TurretConstants.followTagI, TurretConstants.followTagD),
        vision::getCompensatedCenterLimelightX,
        0,
        output -> turretIO.setVoltage(output),
        this);
  }

  public Command lockToSpeakerPose(CommandSwerveDrivetrain swerve, Pose2d speakerPose) {
    return this.run(
        () ->
            this.setPosition(
                swerve
                    .getState()
                    .Pose
                    .rotateBy(
                        getTurretPosition(
                            Rotation2d.fromDegrees(encoderIOInputs1.encoderPositionDegrees),
                            Rotation2d.fromDegrees(encoderIOInputs2.encoderPositionDegrees)))
                    .minus(speakerPose)
                    .getRotation()
                    .plus(
                        getTurretPosition(
                            Rotation2d.fromDegrees(encoderIOInputs1.encoderPositionDegrees),
                            Rotation2d.fromDegrees(encoderIOInputs2.encoderPositionDegrees)))));
  }

  public Command reset() {
    return this.runOnce(
        () -> {
          if (turretIOInputs.turretMotorPosition > TurretConstants.kForwardLimit) {
            turretIO.setPosition(0);
          } else if (turretIOInputs.turretMotorPosition < TurretConstants.kReverseLimit) {
            turretIO.setPosition(0);
          }
        });
  }
}
