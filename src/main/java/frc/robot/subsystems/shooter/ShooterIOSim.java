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
  private final FlywheelSim flywheelSimModel = new FlywheelSim(
      ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(2) : DCMotor.getKrakenX60(2),
      ShooterConstants.kGearingRatio,
      ShooterConstants.kMomentOfInertia);
  private final TalonFXSimState shooterMotorSim;

  public ShooterIOSim() {
    super();
    shooterMotorSim = super.getMotor().getSimState();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // TODO: Don't use super.updateInputs(inputs)
    super.updateInputs(inputs);
    // XXX: Hey since we're using 2 motors, should I be adding the 2 motor's
    // voltages??
    flywheelSimModel.setInput(shooterMotorSim.getMotorVoltage());
    flywheelSimModel.update(LoggedRobot.defaultPeriodSecs);
    double rps = flywheelSimModel.getAngularVelocityRPM() / 60;
    shooterMotorSim.setRotorVelocity(rps);
    shooterMotorSim.addRotorPosition(rps * LoggedRobot.defaultPeriodSecs);
    SimViz.getInstance().addToShooterFlywheelAngle(Math.toDegrees(rps) * LoggedRobot.defaultPeriodSecs);
  }
}
