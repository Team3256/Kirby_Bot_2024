// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevatorrollers;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public double rollerMotorVoltage = 0.0;
    public double rollerMotorVelocity = 0.0;
    public double rollerMotorStatorCurrent = 0.0;
    public double rollerMotorSupplyCurrent = 0.0;
    public double rollerMotorTemperature = 0.0;
    public double rollerMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerVoltage(double voltage) {}

  public default void setRollerVelocity(double velocity) {}

  public default TalonFX getRollerMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getRollerVoltageRequest() {
    return new VoltageOut(0);
  }

  public default void off() {}
}
