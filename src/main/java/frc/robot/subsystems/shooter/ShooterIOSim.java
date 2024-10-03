// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.SimViz;
import org.littletonrobotics.junction.LoggedRobot;

public class ShooterIOSim extends ShooterIOTalonFX {
  private final FlywheelSim leftFlywheelSimModel =
      new FlywheelSim(
          ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
          ShooterConstants.kLeftGearingRatio,
          ShooterConstants.kLeftMomentOfInertia);
  private final FlywheelSim rightFlywheelSimModel =
      new FlywheelSim(
          ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
          ShooterConstants.kRightGearingRatio,
          ShooterConstants.kRightMomentOfInertia);
  private final TalonFXSimState shooterMotorSim;
  private final TalonFXSimState shooterFollowerMotorSim;

  public ShooterIOSim() {
    super();
    shooterMotorSim = super.getMotor().getSimState();
    shooterFollowerMotorSim = super.getFollowerMotor().getSimState();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: Don't use super.updateInputs(inputs)
    super.updateInputs(inputs);
    leftFlywheelSimModel.setInput(shooterMotorSim.getMotorVoltage());
    leftFlywheelSimModel.update(LoggedRobot.defaultPeriodSecs);
    rightFlywheelSimModel.setInput(shooterFollowerMotorSim.getMotorVoltage());
    rightFlywheelSimModel.update(LoggedRobot.defaultPeriodSecs);
    double leftRps = leftFlywheelSimModel.getAngularVelocityRPM() / 60;
    shooterMotorSim.setRotorVelocity(leftRps);
    shooterMotorSim.addRotorPosition(leftRps * LoggedRobot.defaultPeriodSecs);
    double rightRps = rightFlywheelSimModel.getAngularVelocityRPM() / 60;
    shooterFollowerMotorSim.setRotorVelocity(rightRps);
    shooterFollowerMotorSim.addRotorPosition(rightRps * LoggedRobot.defaultPeriodSecs);
    SimViz.getInstance()
        .addToShooterFlywheelAngle(
            Math.toDegrees(leftRps) * LoggedRobot.defaultPeriodSecs,
            Math.toDegrees(rightRps) * LoggedRobot.defaultPeriodSecs);
  }
}
