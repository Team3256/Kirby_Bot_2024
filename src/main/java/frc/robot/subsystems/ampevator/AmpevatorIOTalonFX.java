// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class AmpevatorIOTalonFX implements AmpevatorIO {
  private final TalonFX ampevator = new TalonFX(AmpevatorConstants.ampevatorID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Double> ampevatorVoltage = ampevator.getMotorVoltage();
  private final StatusSignal<Double> ampevatorVelocity = ampevator.getVelocity();
  private final StatusSignal<Double> ampevatorPosition = ampevator.getPosition();
  private final StatusSignal<Double> ampevatorStatorCurrent = ampevator.getStatorCurrent();
  private final StatusSignal<Double> ampevatorSupplyCurrent = ampevator.getSupplyCurrent();
  private final StatusSignal<Double> ampevatorTemperature = ampevator.getDeviceTemp();

  public AmpevatorIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        ampevator, AmpevatorConstants.motorConfigs, AmpevatorConstants.flashConfigRetries);
    BaseStatusSignal.refreshAll(
        ampevatorVoltage,
        ampevatorVelocity,
        ampevatorPosition,
        ampevatorStatorCurrent,
        ampevatorSupplyCurrent,
        ampevatorTemperature);
    ampevator.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(AmpevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        ampevatorVoltage,
        ampevatorVelocity,
        ampevatorPosition,
        ampevatorStatorCurrent,
        ampevatorSupplyCurrent,
        ampevatorTemperature);
    inputs.ampevatorMotorVoltage = ampevatorVoltage.getValueAsDouble();
    inputs.ampevatorMotorVelocity = ampevatorVelocity.getValueAsDouble();
    inputs.ampevatorMotorPosition = ampevatorPosition.getValueAsDouble();
    inputs.ampevatorMotorStatorCurrent = ampevatorStatorCurrent.getValueAsDouble();
    inputs.ampevatorMotorSupplyCurrent = ampevatorSupplyCurrent.getValueAsDouble();
    inputs.ampevatorMotorTemperature = ampevatorTemperature.getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (AmpevatorConstants.useMotionMagic) {
      ampevator.setControl(motionMagicRequest.withPosition(position));
    } else {
      ampevator.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    ampevator.setControl(new VoltageOut(voltage));
  }

  @Override
  public void off() {
    ampevator.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    ampevator.setControl(positionRequest.withPosition(0));
  }
}
