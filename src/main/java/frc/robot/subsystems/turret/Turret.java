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

  /**
   * @see <a
   *     href="https://docs.google.com/spreadsheets/d/1WOmYiwrWg-2iJdFzRZITXqLrtTegc7WbxJ2SQzn8VHs/edit?gid=0#gid=0">Chinese
   *     remainder theorem sheet calculator</a> Uses the calculations from the sheet to calculate
   *     the turret position from two encoders
   * @param cancoder1
   * @param cancoder2
   * @return rotation2d of the turret position
   */
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

  /**
   * @param position Desired position of the turret motor
   * @param swerveAngle Current angle of the swerve drivetrain
   * @return command to set the position of the turret absolute to the swerve angle
   */
  public Command setPositionFieldRelative(Rotation2d position, CommandSwerveDrivetrain swerve) {
    return this.run(
        () ->
            turretIO.setPosition(
                position.minus(swerve.getState().Pose.getRotation()).getRotations()));
  }

  /**
   * @param position Desired position of the turret motor
   * @return Command to set the turret motor to the desired position / by the gear ratio
   */
  public Command setPosition(Rotation2d position) {
    return this.runOnce(() -> turretIO.setPosition(position.getRotations()));
  }

  /**
   * @return Command to zero the turret motor's encoder position
   */
  public Command zero() {
    return this.runOnce(turretIO::zero);
  }

  /**
   * Will run a PID loop that tracks the tx of the speaker tag and attempts to get it to 0 by moving
   * the turret so the LL crosshair is centered
   *
   * @param vision Reference to the vision subsystem
   * @return Command to run a PID loop to lock the turret to the speaker tag
   */
  public Command lockToSpeakerTag(Vision vision) {
    return new PIDCommand(
        new PIDController(
            TurretConstants.followTagP, TurretConstants.followTagI, TurretConstants.followTagD),
        vision::getCompensatedCenterLimelightX,
        0,
        output -> turretIO.setVoltage(output),
        this);
  }

  /**
   * @param swerve Reference to the swerve drivetrain
   * @param speakerPose Pose of the speaker
   * @return Command to turn turret to face the speaker
   */
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

  public Command shakeHead() {
    return setPosition(Rotation2d.fromDegrees(90))
        .andThen(setPosition(Rotation2d.fromDegrees(-90)))
        .repeatedly();
  }

  /**
   * @return Command to reset the turret to the forward limit if it is past the forward limit, or to
   *     the reverse limit if it is past the reverse limit
   */
  public Command reset() {
    return this.runOnce(() -> turretIO.setPosition(0));
  }
}
