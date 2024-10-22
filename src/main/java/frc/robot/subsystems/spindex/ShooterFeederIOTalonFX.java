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

public class ShooterFeederIOTalonFX implements ShooterFeederIO {
  private final TalonFX shooterFeederMotor = new TalonFX(SpindexConstants.shooterFeederMotorID);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVelocityVoltage motionMagicRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);

  private final StatusSignal<Double> shooterFeederMotorVoltage =
      shooterFeederMotor.getMotorVoltage();
  private final StatusSignal<Double> shooterFeederMotorVelocity = shooterFeederMotor.getVelocity();
  private final StatusSignal<Double> shooterFeederMotorStatorCurrent =
      shooterFeederMotor.getStatorCurrent();
  private final StatusSignal<Double> shooterFeederMotorSupplyCurrent =
      shooterFeederMotor.getSupplyCurrent();
  private final StatusSignal<Double> shooterFeederMotorTemperature =
      shooterFeederMotor.getDeviceTemp();

  public ShooterFeederIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        shooterFeederMotor,
        SpindexConstants.feederMotorConfigs,
        SpindexConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        SpindexConstants.updateFrequency,
        shooterFeederMotorVoltage,
        shooterFeederMotorVelocity,
        shooterFeederMotorStatorCurrent,
        shooterFeederMotorSupplyCurrent,
        shooterFeederMotorTemperature);
    shooterFeederMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterFeederIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterFeederMotorVoltage,
        shooterFeederMotorVelocity,
        shooterFeederMotorStatorCurrent,
        shooterFeederMotorSupplyCurrent,
        shooterFeederMotorTemperature);
    inputs.shooterFeederMotorVoltage = shooterFeederMotorVoltage.getValueAsDouble();
    inputs.shooterFeederMotorVelocity = shooterFeederMotorVelocity.getValueAsDouble();
    inputs.shooterFeederMotorStatorCurrent = shooterFeederMotorStatorCurrent.getValueAsDouble();
    inputs.shooterFeederMotorSupplyCurrent = shooterFeederMotorSupplyCurrent.getValueAsDouble();
    inputs.shooterFeederMotorTemperature = shooterFeederMotorTemperature.getValueAsDouble();
  }

  @Override
  public void setFeederVoltage(double voltage) {

    shooterFeederMotor.setVoltage(voltage);
  }

  @Override
  public void setFeederVelocity(double velocity) {
    if (SpindexConstants.useMotionMagic) {
      shooterFeederMotor.setControl(motionMagicRequest.withVelocity(velocity));
    } else {
      shooterFeederMotor.setControl(velocityRequest.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    shooterFeederMotor.setControl(new NeutralOut());
  }
}
