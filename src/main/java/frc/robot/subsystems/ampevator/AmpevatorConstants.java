// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.ampevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class AmpevatorConstants {

  public static final int ampevatorID = 0;
  public static final int ampevatorBeamBreakID = 0;

  public static TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(10)
                  .withKI(0)
                  .withKD(0)
                  .withKG(1)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(50))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static int flashConfigRetries = 5;
  public static boolean useMotionMagic = false;
  public static double ampPreset = 69; // tune pls
  public static double trapPreset = 10; // we never getting this are we
  public static double stowPreset = 0;
}
