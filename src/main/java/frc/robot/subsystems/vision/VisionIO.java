// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    public double ampevatorLimelightX = 0.0;
    public double ampevatorLimelightY = 0.0;

    public double turretLimelightX = 0.0;
    public double turretLimelightY = 0.0;

    public boolean turretLimelightConnected = false;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
