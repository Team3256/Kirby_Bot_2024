// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretConstants {

  public static int kCanCoderID1 = 41;
  public static int kCanCoderID2 = 42;
  public static final CANcoderConfiguration canCoderConfig =
      new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0));
  public static final double updateFrequency = 50.0;

  public static final int kTurretMotorID = 52;

  public static final double followTagP = 1;
  public static final double followTagI = 0;
  public static final double followTagD = 0;

  public static final double kForwardLimit = 32; // TODO: Set limit
  public static final double kReverseLimit = -1; // TODO: Set limit

  public static final Rotation2d kSubPreset = Rotation2d.fromRotations(0);
  public static final Rotation2d kIntakePreset = Rotation2d.fromRotations(16.5);

  public static final TalonFXConfiguration motorConfigs = // TODO: Set configs
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(20)
                  .withKI(0)
                  .withKD(0) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(1600)
                  .withMotionMagicCruiseVelocity(100)
                  .withMotionMagicJerk(3200))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));

  public static final boolean kUseMotionMagic = true;

  public static int flashConfigRetries = 5;

  // CRT constants
  public static final int drivingGear1 = 29;
  public static final int drivenGear1 = 29;
  public static final int drivingGear2 = 29;
  public static final int drivenGear2 = 28;

  public static final double ratio1 = (double) drivingGear1 / drivenGear1;
  public static final double ratio2 = (double) drivingGear2 / drivenGear2;
  public static final double differenceDegrees = (ratio2 - ratio1) * 360;
}
