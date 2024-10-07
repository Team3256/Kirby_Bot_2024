// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import frc.robot.utils.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {

  public VisionIOLimelight() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ampevatorLimelightX = LimelightHelpers.getTX(VisionConstants.ampevatorLimelight);
    inputs.ampevatorLimelightY = LimelightHelpers.getTY(VisionConstants.ampevatorLimelight);

    inputs.turretLimelightX = LimelightHelpers.getTX(VisionConstants.turretLimelight);
    inputs.turretLimelightY = LimelightHelpers.getTY(VisionConstants.turretLimelight);

    inputs.turretLimelightConnected =
        LimelightHelpers.getLatency_Capture(VisionConstants.turretLimelight) != 0;
  }
}
