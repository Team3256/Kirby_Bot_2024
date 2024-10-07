// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevatorrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.PhoenixUtil;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX rollerMotor = new TalonFX(RollerConstants.kRollerMotorID);
  final VelocityVoltage rollerRequest = new VelocityVoltage(0).withSlot(0);
  final MotionMagicVelocityVoltage motionMagicRollerRequest =
      new MotionMagicVelocityVoltage(0).withSlot(0);
  private final VoltageOut rollerVoltageReq = new VoltageOut(0);

  private final StatusSignal<Double> rollerMotorVoltage = rollerMotor.getMotorVoltage();
  private final StatusSignal<Double> rollerMotorVelocity = rollerMotor.getVelocity();
  private final StatusSignal<Double> rollerMotorStatorCurrent = rollerMotor.getStatorCurrent();
  private final StatusSignal<Double> rollerMotorSupplyCurrent = rollerMotor.getSupplyCurrent();
  private final StatusSignal<Double> rollerMotorTemperature = rollerMotor.getDeviceTemp();
  private final StatusSignal<Double> rollerMotorReferenceSlope =
      rollerMotor.getClosedLoopReferenceSlope();

  private DigitalInput beamBreakInput = new DigitalInput(RollerConstants.kRollerBeamBreakDIO);

  public RollerIOTalonFX() {
    PhoenixUtil.applyMotorConfigs(
        rollerMotor, RollerConstants.motorConfigs, RollerConstants.flashConfigRetries);

    BaseStatusSignal.setUpdateFrequencyForAll(
        RollerConstants.updateFrequency,
        rollerMotorVoltage,
        rollerMotorVelocity,
        rollerMotorStatorCurrent,
        rollerMotorSupplyCurrent,
        rollerMotorTemperature,
        rollerMotorReferenceSlope);
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerMotorVoltage,
        rollerMotorVelocity,
        rollerMotorStatorCurrent,
        rollerMotorSupplyCurrent,
        rollerMotorTemperature,
        rollerMotorReferenceSlope);
    inputs.rollerMotorVoltage = rollerMotorVoltage.getValueAsDouble();
    inputs.rollerMotorVelocity = rollerMotorVelocity.getValueAsDouble();
    inputs.rollerMotorStatorCurrent = rollerMotorStatorCurrent.getValueAsDouble();
    inputs.rollerMotorSupplyCurrent = rollerMotorSupplyCurrent.getValueAsDouble();
    inputs.rollerMotorTemperature = rollerMotorTemperature.getValueAsDouble();
    inputs.rollerMotorReferenceSlope = rollerMotorReferenceSlope.getValueAsDouble();

    inputs.isBeamBroken = !beamBreakInput.get();
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  @Override
  public void setRollerVelocity(double velocity) {
    if (RollerConstants.kRollerMotionMagic) {
      rollerMotor.setControl(motionMagicRollerRequest.withVelocity(velocity));
    } else {
      rollerMotor.setControl(rollerRequest.withVelocity(velocity));
    }
  }

  @Override
  public void off() {
    rollerMotor.setControl(new NeutralOut());
  }

  @Override
  public boolean isBeamBroken() {
    return !beamBreakInput.get();
  }

  @Override
  public TalonFX getRollerMotor() {
    return rollerMotor;
  }

  @Override
  public VoltageOut getRollerVoltageRequest() {
    return rollerVoltageReq;
  }
}
