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
          AmpevatorConstants.simGearing,
          AmpevatorConstants.carriageMass,
          AmpevatorConstants.drumRadius,
          AmpevatorConstants.minHeight,
          AmpevatorConstants.maxHeight,
          true,
          AmpevatorConstants.startingHeight);

  private TalonFXSimState ampevatorSimState;

  public AmpevatorIOSim() {
    super();
    ampevatorSimState = super.getMotor().getSimState();
    ampevatorSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void updateInputs(AmpevatorIOInputs inputs) {
    ampevatorSimState = super.getMotor().getSimState();
    ampevatorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    ampevatorSimModel.setInputVoltage(ampevatorSimState.getMotorVoltage());
    ampevatorSimModel.update(LoggedRobot.defaultPeriodSecs);
    ampevatorSimState.setRawRotorPosition(
        ampevatorSimModel.getPositionMeters() * AmpevatorConstants.simGearing);
    ampevatorSimState.setRotorVelocity(
        ampevatorSimModel.getVelocityMetersPerSecond() * AmpevatorConstants.simGearing);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(ampevatorSimModel.getCurrentDrawAmps()));
    inputs.ampevatorMotorVoltage = ampevatorSimState.getMotorVoltage();
    inputs.ampevatorMotorVelocity =
        ampevatorSimModel.getVelocityMetersPerSecond() * AmpevatorConstants.simGearing;
    inputs.ampevatorMotorPosition =
        ampevatorSimModel.getPositionMeters() * AmpevatorConstants.simGearing;
    inputs.ampevatorMotorStatorCurrent = ampevatorSimModel.getCurrentDrawAmps();
    inputs.ampevatorMotorSupplyCurrent = ampevatorSimModel.getCurrentDrawAmps();
    inputs.ampevatorMotorTemperature = 0;

    SimMechs.updateAmpevator(ampevatorSimModel.getPositionMeters());
  }
}
