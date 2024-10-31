// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.generics.SingleMotorConstants;

public final class IntakeConstants implements SingleMotorConstants {
  // Motor configuration
  public static final int kMotorID = 33;
  public static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs().withKS(0).withKV(0.1).withKP(1).withKI(0).withKD(0))
      .withMotorOutput(
          new MotorOutputConfigs()
              .withNeutralMode(NeutralModeValue.Brake)
              .withInverted(InvertedValue.Clockwise_Positive))
      .withMotionMagic(
          new MotionMagicConfigs()
              .withMotionMagicAcceleration(120)
              .withMotionMagicCruiseVelocity(60)
              .withMotionMagicJerk(1200))
      .withCurrentLimits(
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(80));
  public static final boolean kUseMotionMagic = false;
  // Custom to intake subsystem
  public static final double kMotorVoltage = 12;
  public static final int kIntakeBeamBreakDIO = 4;
  public static double kIntakeRedirectVoltage = -2;
  // The rest of these are default
  public static final boolean kUseFOC = true;
  public static final double kStatusSignalUpdateFrequency = 50.0; // Hz
  public static final int kFlashConfigRetries = 5;

  public static class SimulationConstants {
    public static final double kGearRatio = 1.0;
  }
}
