// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.utils.SimViz;
import org.littletonrobotics.junction.LoggedRobot;

public class ShooterIOSim extends ShooterIOTalonFX {
  private final FlywheelSim leftFlywheelSimModel =
      new FlywheelSim(
          ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
          ShooterConstants.SimulationConstants.kLeftGearingRatio,
          ShooterConstants.SimulationConstants.kLeftMomentOfInertia);
  private final FlywheelSim rightFlywheelSimModel =
      new FlywheelSim(
          ShooterConstants.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
          ShooterConstants.SimulationConstants.kRightGearingRatio,
          ShooterConstants.SimulationConstants.kRightMomentOfInertia);
  private final TalonFXSimState shooterMotorSim;
  private final TalonFXSimState shooterFollowerMotorSim;

  public ShooterIOSim() {
    super();
    shooterMotorSim = super.getMotor().getSimState();
    shooterFollowerMotorSim = super.getFollowerMotor().getSimState();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Update battery voltage
    shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    shooterFollowerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    // For Advantage Kit >>>
    inputs.shooterMotorVoltage = shooterMotorSim.getMotorVoltage();
    inputs.shooterMotorVelocity = leftFlywheelSimModel.getAngularVelocityRPM() / 60;
    // In a perfect motor, the supply current and stator current would be equal
    inputs.shooterMotorStatorCurrent = shooterMotorSim.getSupplyCurrent();
    inputs.shooterMotorSupplyCurrent = shooterMotorSim.getSupplyCurrent();
    inputs.shooterMotorTemperature = 0.0; // In a perfect motor, no heat is generated
    // XXX:
    // 1. This could be optimized
    // 2. what about BaseStatusSignal.refreshAll
    // 3. I'm not even sure if this work in sim
    inputs.shooterMotorReferenceSlope =
        super.getMotor().getClosedLoopReferenceSlope().getValueAsDouble();

    inputs.shooterMotorFollowerVoltage = shooterFollowerMotorSim.getMotorVoltage();
    inputs.shooterMotorFollowerVelocity = rightFlywheelSimModel.getAngularVelocityRPM() / 60;
    inputs.shooterMotorFollowerStatorCurrent = shooterFollowerMotorSim.getSupplyCurrent();
    inputs.shooterMotorFollowerSupplyCurrent = shooterFollowerMotorSim.getSupplyCurrent();
    inputs.shooterMotorFollowerTemperature = 0.0;
    inputs.shooterMotorFollowerReferenceSlope =
        super.getFollowerMotor().getClosedLoopReferenceSlope().getValueAsDouble();
    // <<< For Advantage Kit
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
            Math.toDegrees(leftRps)
                * LoggedRobot.defaultPeriodSecs
                * ShooterConstants.SimulationConstants.kAngularVelocityScalar,
            Math.toDegrees(rightRps)
                * LoggedRobot.defaultPeriodSecs
                * ShooterConstants.SimulationConstants.kAngularVelocityScalar);

    // Update battery voltage
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            leftFlywheelSimModel.getCurrentDrawAmps(), rightFlywheelSimModel.getCurrentDrawAmps()));
  }
}
