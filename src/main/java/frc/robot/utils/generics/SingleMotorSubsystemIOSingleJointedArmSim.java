// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.LoggedRobot;

public class SingleMotorSubsystemIOSingleJointedArmSim<T extends SingleJointedConstants>
    extends SingleMotorSubsystemIOTalonFX<T> {
  private TalonFXSimState motorSim;
  protected final SingleJointedArmSim jointedModel =
      new SingleJointedArmSim(
          T.kUseFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
          T.SimulationConstants.kGearRatio,
          T.SimulationConstants.kMomentOfInertia,
          T.SimulationConstants.kArmLength.in(Meters),
          T.SimulationConstants.kMinAngle.in(Radians),
          T.SimulationConstants.kMaxAngle.in(Radians),
          true,
          T.SimulationConstants.kStartingAngle.in(Radians));

  public SingleMotorSubsystemIOSingleJointedArmSim() {
    super();
    this.motorSim = super.getMotor().getSimState();
  }

  public void updateInputs(SingleMotorSubsystemInputs inputs) {
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    jointedModel.setInputVoltage(motorSim.getMotorVoltage());
    jointedModel.update(LoggedRobot.defaultPeriodSecs);
    motorSim.setRawRotorPosition(
        Units.radiansToRotations(jointedModel.getAngleRads()) * T.SimulationConstants.kGearRatio);
    motorSim.setRotorVelocity(
        Units.radiansToRotations(jointedModel.getVelocityRadPerSec())
            * T.SimulationConstants.kGearRatio);
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(jointedModel.getCurrentDrawAmps()));
    super.updateInputs(inputs);
  }

  public Measure<Angle> getAngle() {
    return Radians.of(jointedModel.getAngleRads());
  }

  public double getAngleRads() {
    return jointedModel.getAngleRads();
  }
}
