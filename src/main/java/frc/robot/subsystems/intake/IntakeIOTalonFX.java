// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID);
  final VelocityVoltage intakeRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicIntakeRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut intakeVoltageReq = new VoltageOut(0);

  private final StatusSignal<Double> intakeMotorVoltage = intakeMotor.getMotorVoltage();
  private final StatusSignal<Double> intakeMotorVelocity = intakeMotor.getVelocity();
  private final StatusSignal<Double> intakeMotorStatorCurrent = intakeMotor.getStatorCurrent();
  private final StatusSignal<Double> intakeMotorSupplyCurrent = intakeMotor.getSupplyCurrent();
  private final StatusSignal<Double> intakeMotorTemperature = intakeMotor.getDeviceTemp();
  private final StatusSignal<Double> intakeMotorReferenceSlope =
      intakeMotor.getClosedLoopReferenceSlope();

  public IntakeIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        intakeMotor, IntakeConstants.motorConfigs, (int) IntakeConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        IntakeConstants.updateFrequency,
        intakeMotorVoltage,
        intakeMotorVelocity,
        intakeMotorStatorCurrent,
        intakeMotorSupplyCurrent,
        intakeMotorTemperature,
        intakeMotorReferenceSlope);
    intakeMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        intakeMotorVoltage,
        intakeMotorVelocity,
        intakeMotorStatorCurrent,
        intakeMotorSupplyCurrent,
        intakeMotorTemperature,
        intakeMotorReferenceSlope);
    inputs.intakeMotorVoltage = intakeMotorVoltage.getValueAsDouble();
    inputs.intakeMotorVelocity = intakeMotorVelocity.getValueAsDouble();
    inputs.intakeMotorStatorCurrent = intakeMotorStatorCurrent.getValueAsDouble();
    inputs.intakeMotorSupplyCurrent = intakeMotorSupplyCurrent.getValueAsDouble();
    inputs.intakeMotorTemperature = intakeMotorTemperature.getValueAsDouble();
    inputs.intakeMotorReferenceSlope = intakeMotorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeVelocity(double velocity) {
    if (IntakeConstants.kIntakeMotionMagic) {
      intakeMotor.setControl(motionMagicIntakeRequest.withVelocity(velocity));
    } else {
      intakeMotor.setControl(intakeRequest.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    intakeMotor.setControl(new NeutralOut());
  }

  @Override
  public TalonFX getIntakeMotor() {
    return intakeMotor;
  }

  @Override
  public VoltageOut getIntakeVoltageRequest() {
    return intakeVoltageReq;
  }
}
