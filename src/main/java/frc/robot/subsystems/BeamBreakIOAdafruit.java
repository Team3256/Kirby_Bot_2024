// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOAdafruit implements BeamBreakIO {
  private final DigitalInput beamBreak;

  public BeamBreakIOAdafruit(int beambreakID) {
    beamBreak = new DigitalInput(beambreakID);
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.beamBroken = !beamBreak.get(); // i lowk forgot if this is inverted or not
  }
}
