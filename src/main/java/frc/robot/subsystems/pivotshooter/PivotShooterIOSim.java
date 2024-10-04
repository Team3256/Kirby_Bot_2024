// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class PivotShooterIOSim extends PivotShooterIOTalonFX {

  private final SingleJointedArmSim pivotShooterSimModel =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          PivotShooterConstants.sim.simGearing,
          PivotShooterConstants.sim.jkGMetersSquared,
          PivotShooterConstants.sim.armLength,
          PivotShooterConstants.sim.minAngle.getRadians(),
          PivotShooterConstants.sim.maxAngle.getRadians(),
          true,
          PivotShooterConstants.sim.startingAngle.getRadians());

  private TalonFXSimState pivotShooterSimState;

  public PivotShooterIOSim() {
    super();
    pivotShooterSimState = super.getMotor().getSimState();
    pivotShooterSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(PivotShooterIOInputs inputs) {


    pivotShooterSimState = super.getMotor().getSimState();
    pivotShooterSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    pivotShooterSimModel.setInputVoltage(pivotShooterSimState.getMotorVoltage());
    pivotShooterSimModel.update(LoggedRobot.defaultPeriodSecs);
    pivotShooterSimState.setRawRotorPosition(
        Units.radiansToRotations(pivotShooterSimModel.getAngleRads())
            * PivotShooterConstants.sim.simGearing);
    pivotShooterSimState.setRotorVelocity(
        Units.radiansToRotations(pivotShooterSimModel.getVelocityRadPerSec())
            * PivotShooterConstants.sim.simGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(pivotShooterSimModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
    System.out.println(pivotShooterSimModel.getAngleRads());
    SimMechs.updatePivotShooter(Rotation2d.fromRadians(pivotShooterSimModel.getAngleRads()));
  }
}
