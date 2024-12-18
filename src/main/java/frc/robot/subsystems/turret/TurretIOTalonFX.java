// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicExpoVoltage motionMagicRequest =
      new MotionMagicExpoVoltage(0).withSlot(0);

  private final StatusSignal<Double> turretMotorVoltage = turretMotor.getMotorVoltage();
  private final StatusSignal<Double> turretMotorRotorPos = turretMotor.getRotorPosition();
  private final StatusSignal<Double> turretMotorPosition = turretMotor.getPosition();
  private final StatusSignal<Double> turretMotorStatorCurrent = turretMotor.getStatorCurrent();
  private final StatusSignal<Double> turretMotorSupplyCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Double> turretMotorTemperature = turretMotor.getDeviceTemp();
  private final StatusSignal<Double> turretMotorReferenceSlope =
      turretMotor.getClosedLoopReferenceSlope();

  public TurretIOTalonFX() {
    var motorConfig = TurretConstants.motorConfigs;
    ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
    closedLoopGeneralConfigs.ContinuousWrap = false;
    motorConfig.ClosedLoopGeneral = closedLoopGeneralConfigs;
    PhoenixUtil.applyMotorConfigs(turretMotor, motorConfig, TurretConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.updateFrequency,
        turretMotorRotorPos,
        turretMotorVoltage,
        turretMotorPosition,
        turretMotorStatorCurrent,
        turretMotorSupplyCurrent,
        turretMotorTemperature,
        turretMotorReferenceSlope);
    turretMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        turretMotorVoltage,
        turretMotorPosition,
        turretMotorRotorPos,
        turretMotorStatorCurrent,
        turretMotorSupplyCurrent,
        turretMotorTemperature,
        turretMotorReferenceSlope);
    inputs.turretMotorVoltage = turretMotorVoltage.getValueAsDouble();
    inputs.turretMotorPosition = turretMotorPosition.getValueAsDouble();
    inputs.turretMotorRotorPos = turretMotorRotorPos.getValueAsDouble();
    inputs.turretMotorStatorCurrent = turretMotorStatorCurrent.getValueAsDouble();
    inputs.turretMotorSupplyCurrent = turretMotorSupplyCurrent.getValueAsDouble();
    inputs.turretMotorTemperature = turretMotorTemperature.getValueAsDouble();
    inputs.turretMotorReferenceSlope = turretMotorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public void setPosition(double position) {
    if (TurretConstants.kUseMotionMagic) {
      turretMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      turretMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void off() {
    turretMotor.setControl(new NeutralOut());
  }

  @Override
  public void setSensorPosition(double position) {
    turretMotor.setPosition(position);
  }

  @Override
  public void zero() {
    turretMotor.setPosition(0);
  }
}
