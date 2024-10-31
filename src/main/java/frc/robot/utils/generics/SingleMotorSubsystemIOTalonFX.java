// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils.generics;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class SingleMotorSubsystemIOTalonFX<T extends SingleMotorConstants>
    implements SingleMotorSubsystemIO {

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> motorVoltage;
  private final StatusSignal<Double> motorVelocity;
  private final StatusSignal<Double> motorPosition;
  private final StatusSignal<Double> motorStatorCurrent;
  private final StatusSignal<Double> motorSupplyCurrent;
  private final StatusSignal<Double> motorTemperature;
  private final StatusSignal<Double> motorReferenceSlope;

  public SingleMotorSubsystemIOTalonFX() {
    motor = new TalonFX(T.kMotorID);
    motorVoltage = motor.getMotorVoltage();
    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();
    motorStatorCurrent = motor.getStatorCurrent();
    motorSupplyCurrent = motor.getSupplyCurrent();
    motorTemperature = motor.getDeviceTemp();
    motorReferenceSlope = motor.getClosedLoopReferenceSlope();

    PhoenixUtil.applyMotorConfigs(motor, T.kMotorConfig, T.kFlashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        T.kStatusSignalUpdateFrequency,
        motorVoltage,
        motorVelocity,
        motorPosition,
        motorStatorCurrent,
        motorSupplyCurrent,
        motorTemperature,
        motorReferenceSlope);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SingleMotorSubsystemInputs inputs) {
    BaseStatusSignal.refreshAll(
        motorVoltage,
        motorVelocity,
        motorPosition,
        motorStatorCurrent,
        motorSupplyCurrent,
        motorTemperature,
        motorReferenceSlope);
    inputs.motorVoltage = motorVoltage.getValueAsDouble();
    inputs.motorVelocity = motorVelocity.getValueAsDouble();
    inputs.motorPosition = motorPosition.getValueAsDouble();
    inputs.motorStatorCurrent = motorStatorCurrent.getValueAsDouble();
    inputs.motorSupplyCurrent = motorSupplyCurrent.getValueAsDouble();
    inputs.motorTemperature = motorTemperature.getValueAsDouble();
    inputs.motorReferenceSlope = motorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (T.kUseMotionMagic) {
      motor.setControl(motionMagicRequest.withPosition(position));
    } else {
      motor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void off() {
    motor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    motor.setPosition(0);
  }

  @Override
  public TalonFX getMotor() {
    return motor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
