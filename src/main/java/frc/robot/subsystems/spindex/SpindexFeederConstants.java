// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.spindex;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.utils.generics.SingleMotorConstants;

public class SpindexFeederConstants extends BaseSpindexConstants implements SingleMotorConstants {
  public static int kMotorID = 43;
  public static TalonFXConfiguration kMotorConfig =
      new TalonFXConfiguration() // TODO: tune
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
  ;

  public static double shooterFeederVoltage = 8;

  public static class SimulationConstants {
    public static final double kGearRatio = 1.0;
  }
}
