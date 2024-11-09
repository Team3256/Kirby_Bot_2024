// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionIOAutoLogged = new VisionIOInputsAutoLogged();

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(visionIOAutoLogged);
    Logger.processInputs(this.getClass().getSimpleName(), visionIOAutoLogged);
  }

  public double getCenterLimelightX() {
    return visionIOAutoLogged.turretLimelightX;
  }

  public double getCenterLimelightY() {
    return visionIOAutoLogged.turretLimelightY;
  }

  @AutoLogOutput
  public double getDistanceToNote() {
    return -(VisionConstants.noteLimelightHeightInches - VisionConstants.noteHeightInches)
        / Math.tan(
            Units.degreesToRadians(
                visionIOAutoLogged.ampevatorLimelightY
                    + VisionConstants.noteLimelightAngleDegrees));
  }

  public Pose2d getNotePose(Pose2d robotPose) {
    return robotPose
        .transformBy(VisionConstants.robotToCam)
        .transformBy(
            new Transform2d(
                new Translation2d(
                    Units.inchesToMeters(getDistanceToNote()),
                    Rotation2d.fromDegrees(visionIOAutoLogged.ampevatorLimelightX)),
                Rotation2d.fromDegrees(visionIOAutoLogged.ampevatorLimelightX)));
  }
}
