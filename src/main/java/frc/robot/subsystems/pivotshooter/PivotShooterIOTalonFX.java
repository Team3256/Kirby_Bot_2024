// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class PivotShooterIOTalonFX implements PivotShooterIO {

  private final TalonFX pivotShooterMotor = new TalonFX(PivotShooterConstants.kPivotMotorID);
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withSlot(0).withEnableFOC(PivotShooterConstants.kUseFOC);
  private final MotionMagicVoltage motionMagicRequest =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(PivotShooterConstants.kUseFOC);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> pivotShooterMotorVoltage = pivotShooterMotor.getMotorVoltage();
  private final StatusSignal<Double> pivotShooterMotorVelocity = pivotShooterMotor.getVelocity();
  private final StatusSignal<Double> pivotShooterMotorPosition = pivotShooterMotor.getPosition();
  private final StatusSignal<Double> pivotShooterMotorStatorCurrent =
      pivotShooterMotor.getStatorCurrent();
  private final StatusSignal<Double> pivotShooterMotorSupplyCurrent =
      pivotShooterMotor.getSupplyCurrent();
  private final StatusSignal<Double> pivotShooterMotorTemperature =
      pivotShooterMotor.getDeviceTemp();

  public PivotShooterIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        pivotShooterMotor,
        PivotShooterConstants.motorConfigs,
        PivotShooterConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotShooterConstants.updateFrequency,
        pivotShooterMotorVoltage,
        pivotShooterMotorVelocity,
        pivotShooterMotorPosition,
        pivotShooterMotorStatorCurrent,
        pivotShooterMotorSupplyCurrent,
        pivotShooterMotorTemperature);
    pivotShooterMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(PivotShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotShooterMotorVoltage,
        pivotShooterMotorVelocity,
        pivotShooterMotorPosition,
        pivotShooterMotorStatorCurrent,
        pivotShooterMotorSupplyCurrent,
        pivotShooterMotorTemperature);
    inputs.pivotShooterMotorVoltage = pivotShooterMotorVoltage.getValueAsDouble();
    inputs.pivotShooterMotorVelocity = pivotShooterMotorVelocity.getValueAsDouble();
    inputs.pivotShooterMotorPosition = pivotShooterMotorPosition.getValueAsDouble();
    inputs.pivotShooterMotorStatorCurrent = pivotShooterMotorStatorCurrent.getValueAsDouble();
    inputs.pivotShooterMotorSupplyCurrent = pivotShooterMotorSupplyCurrent.getValueAsDouble();
    inputs.pivotShooterMotorTemperature = pivotShooterMotorTemperature.getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (PivotShooterConstants.kUseMotionMagic) {
      pivotShooterMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      pivotShooterMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    pivotShooterMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    pivotShooterMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    pivotShooterMotor.setPosition(0);
  }

  @Override
  public TalonFX getMotor() {
    return pivotShooterMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
