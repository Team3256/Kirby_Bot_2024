// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.utils.PhoenixUtil;

public class ClimbIOTalonFX implements ClimbIO {

  private final TalonFX climbMotor = new TalonFX(ClimbConstants.kLeftClimbMotorID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VoltageOut voltageReq = new VoltageOut(0);

  private final StatusSignal<Double> climbMotorVoltage = climbMotor.getMotorVoltage();
  private final StatusSignal<Double> climbMotorVelocity = climbMotor.getVelocity();
  private final StatusSignal<Double> climbMotorPosition = climbMotor.getPosition();
  private final StatusSignal<Double> climbMotorStatorCurrent = climbMotor.getStatorCurrent();
  private final StatusSignal<Double> climbMotorSupplyCurrent = climbMotor.getSupplyCurrent();
  private final StatusSignal<Double> climbMotorTemperature = climbMotor.getDeviceTemp();
  private final StatusSignal<Double> climbMotorReferenceSlope =
      climbMotor.getClosedLoopReferenceSlope();

  public ClimbIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        climbMotor, ClimbConstants.motorConfig, ClimbConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        ClimbConstants.updateFrequency,
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        climbMotorVoltage,
        climbMotorVelocity,
        climbMotorPosition,
        climbMotorStatorCurrent,
        climbMotorSupplyCurrent,
        climbMotorTemperature,
        climbMotorReferenceSlope);
    inputs.climbMotorVoltage = climbMotorVoltage.getValueAsDouble();
    inputs.climbMotorVelocity = climbMotorVelocity.getValueAsDouble();
    inputs.climbMotorPosition = climbMotorPosition.getValueAsDouble();
    inputs.climbMotorStatorCurrent = climbMotorStatorCurrent.getValueAsDouble();
    inputs.climbMotorSupplyCurrent = climbMotorSupplyCurrent.getValueAsDouble();
    inputs.climbMotorTemperature = climbMotorTemperature.getValueAsDouble();
    inputs.climbMotorReferenceSlope = climbMotorReferenceSlope.getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (ClimbConstants.kUseMotionMagic) {
      climbMotor.setControl(motionMagicRequest.withPosition(position));
    } else {
      climbMotor.setControl(positionRequest.withPosition(position));
    }
  }

  @Override
  public void setVoltage(double voltage) {
    climbMotor.setVoltage(voltage);
  }

  @Override
  public void off() {
    climbMotor.setControl(new NeutralOut());
  }

  @Override
  public void zero() {
    climbMotor.setPosition(0);
  }

  @Override
  public TalonFX getMotor() {
    return climbMotor;
  }

  @Override
  public VoltageOut getVoltageRequest() {
    return voltageReq;
  }
}
