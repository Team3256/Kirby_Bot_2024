// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class SpindexIOTalonFX implements SpindexIO {
  private final TalonFX spindexMotor = new TalonFX(SpindexConstants.spindexMotorID);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<Double> spindexMotorVoltage = spindexMotor.getMotorVoltage();
  private final StatusSignal<Double> spindexMotorVelocity = spindexMotor.getVelocity();
  private final StatusSignal<Double> spindexMotorStatorCurrent = spindexMotor.getStatorCurrent();
  private final StatusSignal<Double> spindexMotorSupplyCurrent = spindexMotor.getSupplyCurrent();
  private final StatusSignal<Double> spindexMotorTemperature = spindexMotor.getDeviceTemp();

  public SpindexIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        spindexMotor, SpindexConstants.motorConfigs, SpindexConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        SpindexConstants.updateFrequency,
        spindexMotorVoltage,
        spindexMotorVelocity,
        spindexMotorStatorCurrent,
        spindexMotorSupplyCurrent,
        spindexMotorTemperature);
    spindexMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SpindexIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        spindexMotorVoltage,
        spindexMotorVelocity,
        spindexMotorStatorCurrent,
        spindexMotorSupplyCurrent,
        spindexMotorTemperature);
    inputs.spindexMotorVoltage = spindexMotorVoltage.getValueAsDouble();
    inputs.spindexMotorVelocity = spindexMotorVelocity.getValueAsDouble();
    inputs.spindexMotorStatorCurrent = spindexMotorStatorCurrent.getValueAsDouble();
    inputs.spindexMotorSupplyCurrent = spindexMotorSupplyCurrent.getValueAsDouble();
    inputs.spindexMotorTemperature = spindexMotorTemperature.getValueAsDouble();
  }

  @Override
  public void setSpindexVoltage(double voltage) {

    spindexMotor.setVoltage(voltage);
  }

  @Override
  public void setSpindexVelocity(double velocity) {
    if (SpindexConstants.useMotionMagic) {
      spindexMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      spindexMotor.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    spindexMotor.setControl(new NeutralOut());
  }
}
