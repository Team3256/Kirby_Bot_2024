// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivotshooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.robot.utils.generics.SingleJointedConstants;

public final class PivotShooterConstants implements SingleJointedConstants {

  public static final int kMotorID = 60;
  public static final TalonFXConfiguration kMotorConfig =
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

  public static final boolean kUseMotionMagic = true; // idk
  public static final boolean kUseFOC = false;
  public static final double kStatusSignalUpdateFrequency = 50.0;
  public static final int kFlashConfigRetries = 5;
  // Presets >>
  // public static final double kSubWooferPreset = (3.5 + 0.3) / 138.33;
  public static final double kSubWooferPreset = 3.4 / 138.33; // 3.2
  public static final double kFeederPreset = 5.9 / 138.33;
  public static final double kAmpPreset = (4) / 138.33;
  public static final double kWingNoteCenterPreset = 5.8 / 138.33;
  public static final double kWingNoteSidePreset =
      5.5 / 138.33; // old value: 5.7 distance: -1.5 //old ish?: 5.4
  public static final double kWingNoteFarSidePreset = 0 / 138.33;
  public static final double kTrussSourceSidePreset = 6.7 / 138.33; // -10.6875
  public static final double kHalfWingPodiumPreset =
      6.55 / 138.33; // old value: 6.7 distance: -11.5275
  public static final double kPodiumLeftPreset = 6.5 / 138.33;
  public static final double kPodiumRPreset = 6 / 138.33;

  public static final double kPivotSlamStallCurrent = 50;
  public static final double kPivotSlamShooterVoltage = -2;

  // public static final double kPivotSlamIntakeVoltage = -5;

  // public static final double kStallVelocityThreshold = 0.1;
  // public static final double kPivotPositionToleranceDeg = 0.1; // 5deg for the
  // << Presets

  /* PID */
  // public static final TrapezoidProfile.Constraints kPivotProfileContraints =
  // new TrapezoidProfile.Constraints(16, 16);

  public static final class SimulationConstants {
    public static final double kGearRatio = 138.333; // 22 by 1
    public static final Measure<Distance> kArmLength = Meters.of(0.2);
    public static final double kMomentOfInertia = 0.1;
    // SingleJointedArmSim.estimateMOI(
    // kArmLength.in(Meters), Kilograms.of(2).magnitude()); // Or moi = 0.1??
    public static final Measure<Angle> kMinAngle = Degrees.of(-90);
    public static final Measure<Angle> kMaxAngle = Degrees.of(50);
    public static final boolean kSimulateGravity = true;
    public static final Measure<Angle> kStartingAngle = Degrees.of(0);
  }
}
