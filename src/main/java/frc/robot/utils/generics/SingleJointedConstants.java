// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;

public interface SingleJointedConstants extends SingleMotorConstants {
  public static class SimulationConstants {

    public static final double kGearRatio = 1.0;
    public static final Measure<Distance> kArmLength = Meters.of(0); // needs to be tuned
    public static final double kMomentOfInertia = 1; // in kilogram * meter^2
    public static final Measure<Angle> kMinAngle = Degrees.of(0); // needs to be tuned
    public static final Measure<Angle> kMaxAngle = Degrees.of(0); // needs to be tuned
    public static final boolean kSimulateGravity = true;
    public static final Measure<Angle> kStartingAngle = Degrees.of(0); // needs to be tuned
  }
}
