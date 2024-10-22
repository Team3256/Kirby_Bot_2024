// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretConstants {

  public static int kCanCoderID1 = 0; // TODO: set id
  public static int kCanCoderID2 = 0; // TODO: set id

  public static final double gearRatio = 1; // TODO: Set gear ratio
  public static final CANcoderConfiguration canCoderConfig =
      new CANcoderConfiguration()
          .withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0)); // TODO: set config
  public static final double updateFrequency = 50.0;

  public static final int kTurretMotorID = 0; // TODO: Set ID

  public static final double followTagP = 1;
  public static final double followTagI = 0;
  public static final double followTagD = 0;

  public static final double kForwardLimit = 69; // TODO: Set limit
  public static final double kReverseLimit = -69; // TODO: Set limit

  public static final Rotation2d kSubPreset = Rotation2d.fromRotations(0);

  public static final TalonFXConfiguration motorConfigs = // TODO: Set configs
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0.05)
                  .withKP(25)
                  .withKI(0)
                  .withKD(0) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(100)
                  .withMotionMagicCruiseVelocity(100)
                  .withMotionMagicJerk(420))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(kForwardLimit)
                  .withReverseSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(kReverseLimit));

  public static final boolean kUseMotionMagic = false;

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
