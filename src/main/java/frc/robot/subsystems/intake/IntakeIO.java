// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorVelocity = 0.0;
    public double intakeMotorStatorCurrent = 0.0;
    public double intakeMotorSupplyCurrent = 0.0;
    public double intakeMotorTemperature = 0.0;
    public double intakeMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeVoltage(double voltage) {}

  public default void setIntakeVelocity(double velocity) {}

  public default TalonFX getIntakeMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getIntakeVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}
}
