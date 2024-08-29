// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface AmpevatorIO {
  @AutoLog
  public static class AmpevatorIOInputs {
    public double ampevatorMotorVoltage = 0.0;
    public double ampevatorMotorPosition = 0.0;
    public double ampevatorMotorVelocity = 0.0;
    public double ampevatorMotorStatorCurrent = 0.0;
    public double ampevatorMotorSupplyCurrent = 0.0;
    public double ampevatorMotorTemperature = 0.0;
    public double ampevatorMotorReferenceSlope = 0.0;
  }

  public default void updateInputs(AmpevatorIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default void off() {}

  public default void zero() {}

  public default TalonFX getMotor() {
    return new TalonFX(0);
  }

  public default VoltageOut getVoltageRequest() {
    return new VoltageOut(0);
  }
}
