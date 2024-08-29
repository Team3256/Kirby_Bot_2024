// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class AmpevatorConstants {
  // ampevator
  public static final int kAmpevatorMotorID = -1;
  public static final double gearRatio = -1; // needs to be tuned

  // ampevator positions
  public static final double kAmpevatorUpPosition = -1;
  public static final double kAmpevatorDownPosition = 0;

  public static final TalonFXConfiguration motorConfig =
      new TalonFXConfiguration()
          .withSlot0(new Slot0Configs().withKS(0).withKV(0).withKP(-1).withKI(0).withKD(-1))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      NeutralModeValue.Brake) // ampevator needs to be locked when not moving
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(100))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static final int flashConfigRetries = 5;
}
