// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import org.littletonrobotics.junction.LoggedRobot;

public class SingleMotorSubsystemIOSim<T extends SingleMotorConstants>
    extends SingleMotorSubsystemIOTalonFX<T> {
  private TalonFXSimState motorSim;
  private final DCMotor motorModel =
      (T.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1))
          .withReduction(T.SimulationConstants.kGearRatio);

  public SingleMotorSubsystemIOSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
  }

  public void updateInputs(SingleMotorSubsystemInputs inputs) {
    // Update battery voltage
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double rps =
        motorModel.getSpeed(
                motorModel.getTorque(motorSim.getTorqueCurrent()), motorSim.getMotorVoltage())
            / 2
            * Math.PI; // Convert to RPS
    motorSim.setRotorVelocity(rps);
    motorSim.addRotorPosition(rps * LoggedRobot.defaultPeriodSecs);

    // Update battery voltage (after the effects of physics models)
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            // XXX: not sure the difference between this and getting the current
            // from the motor sim
            motorModel.getCurrent(rps, motorSim.getMotorVoltage())));
    super.updateInputs(inputs);

    // TODO: Vis
  }
}
