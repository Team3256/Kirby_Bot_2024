// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevator;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.sim.SimMechs;
import org.littletonrobotics.junction.LoggedRobot;

public class AmpevatorIOSim extends AmpevatorIOTalonFX {

  private final ElevatorSim ampevatorSimModel =
      new ElevatorSim(
          DCMotor.getKrakenX60(1),
          AmpevatorConstants.sim.simGearing,
          AmpevatorConstants.sim.carriageMass,
          AmpevatorConstants.sim.drumRadius,
          AmpevatorConstants.sim.minHeight,
          AmpevatorConstants.sim.maxHeight,
          true,
          AmpevatorConstants.sim.startingHeight);

  private final TalonFXSimState ampevatorSimState;

  public AmpevatorIOSim() {
    super();
    ampevatorSimState = super.getMotor().getSimState();
    ampevatorSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(AmpevatorIOInputs inputs) {
    super.updateInputs(inputs);

    ampevatorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    ampevatorSimModel.setInputVoltage(ampevatorSimState.getMotorVoltage());
    ampevatorSimModel.update(LoggedRobot.defaultPeriodSecs);
    ampevatorSimState.setRawRotorPosition(
        ampevatorSimModel.getPositionMeters() * AmpevatorConstants.sim.simGearing);
    ampevatorSimState.setRotorVelocity(
        ampevatorSimModel.getVelocityMetersPerSecond() * AmpevatorConstants.sim.simGearing);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(ampevatorSimModel.getCurrentDrawAmps()));

    SimMechs.updateAmpevator(ampevatorSimModel.getPositionMeters());
  }
}
