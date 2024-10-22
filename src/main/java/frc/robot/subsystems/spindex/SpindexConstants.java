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

public class SpindexConstants {
  public static final int spindexMotorID = 0;
  public static final double spindexMotorSpeedRPS = 0.8;
  public static TalonFXConfiguration spindexMotorConfigs =
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
  public static int flashConfigRetries = 5;
  public static double updateFrequency = 50; // idk if this is right
  public static boolean useMotionMagic = false;
  public static int kSpindexBeamBreakDIO = 1;
    public static TalonFXConfiguration feederMotorConfigs = new TalonFXConfiguration() // TODO: tune
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
                            .withStatorCurrentLimit(80));;
  public static int shooterFeederMotorID = 43;
  public static double shooterFeederVoltage = 8;
}
