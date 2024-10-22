// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterFeederIO {
  @AutoLog
  public static class ShooterFeederIOInputs {
    public double shooterFeederMotorVoltage = 0.0;
    public double shooterFeederMotorVelocity = 0.0;
    public double shooterFeederMotorStatorCurrent = 0.0;
    public double shooterFeederMotorSupplyCurrent = 0.0;
    public double shooterFeederMotorTemperature = 0.0;
  }

  public default void updateInputs(ShooterFeederIOInputs inputs) {}

  public default void setFeederVoltage(double voltage) {}

  public default void setFeederVelocity(double velocity) {}

  public default void off() {}
}
