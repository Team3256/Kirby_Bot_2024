// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class PivotShooterConstants {
  // public static final double kSubWooferPreset = (3.5 + 0.3) / 138.33; // idk if
  // this works
  public static final double kSubWooferPreset = 3.4 / 138.33; // 3.2
  public static final double kFeederPreset = 5.9 / 138.33;

  public static final int kPivotMotorID = 60;

  /* PID */

  /* Physics/geometry */
  public static final double kPivotMotorGearing = 138.333; // 22 by 1

  /* Preset */
  public static final double kPivotSlamIntakeVoltage = -5;
  public static final double kPivotSlamShooterVoltage = -2;

  /* Misc */
  public static final boolean kUseFOC = false;
  public static final boolean kUseMotionMagic = true; // idk
  public static final double updateFrequency = 50.0;
  public static final int flashConfigRetries = 5;
  public static final double kPivotSlamStallCurrent = 50;

  public static final TalonFXConfiguration motorConfigs =
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

  public static final class sim {
    public static final double simGearing = 62.67;

    public static final double armLength = .5;
    public static final double jkGMetersSquared = SingleJointedArmSim.estimateMOI(armLength, 2);

    public static final Rotation2d minAngle = Rotation2d.fromDegrees(0);
    public static final Rotation2d maxAngle = Rotation2d.fromDegrees(90);
    public static final Rotation2d startingAngle = Rotation2d.fromDegrees(25);
  }
}
