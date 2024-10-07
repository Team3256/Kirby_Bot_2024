// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakIOInputs {
    public boolean beamBroken = false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}
}
