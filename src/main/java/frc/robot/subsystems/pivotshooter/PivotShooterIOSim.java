// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sim.SimMechs;
import frc.robot.utils.generics.SingleMotorSubsystemIOSingleJointedArmSim;

public class PivotShooterIOSim
    extends SingleMotorSubsystemIOSingleJointedArmSim<PivotShooterConstants> {
  public PivotShooterIOSim() {
    super();
  }

  @Override
  public void updateInputs(SingleMotorSubsystemInputs inputs) {
    super.updateInputs(inputs);
    SimMechs.updatePivotShooter(Rotation2d.fromRadians(jointedModel.getAngleRads()));
  }
}
