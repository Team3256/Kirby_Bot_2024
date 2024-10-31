// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import frc.robot.utils.SingleMotorSim;

public class SpindexIOSim extends SpindexIOTalonFX {
  private final SingleMotorSim motorSim;

  public SpindexIOSim() {
    super();
    motorSim =
        new SingleMotorSim(
            super.getMotor().getSimState(),
            SpindexConstants.SimulationConstants.kGearRatio,
            SpindexConstants.kUseFOC);
  }

  public void updateInputs(SpindexIOInputs inputs) {
    motorSim.updateSim();
    super.updateInputs(inputs);
    // SimMechs.addToShooterFlywheelAngle(
    // Math.toDegrees(leftRps)
    // * LoggedRobot.defaultPeriodSecs
    // * ShooterConstants.SimulationConstants.kAngularVelocityScalar,
    // Math.toDegrees(rightRps)
    // * LoggedRobot.defaultPeriodSecs
    // * ShooterConstants.SimulationConstants.kAngularVelocityScalar);
  }
}
