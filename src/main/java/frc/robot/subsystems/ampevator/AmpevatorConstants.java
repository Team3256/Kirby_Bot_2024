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
import edu.wpi.first.math.util.Units;

public final class AmpevatorConstants {

  public static final int ampevatorID = 47;
  public static final TalonFXConfiguration motorConfigs =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(0)
                  .withKV(0)
                  .withKP(1)
                  .withKI(0)
                  .withKD(0)
                  .withKG(1)
                  .withGravityType(GravityTypeValue.Arm_Cosine) // Original 0.145
              )
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(400)
                  .withMotionMagicCruiseVelocity(50))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60));
  public static final int flashConfigRetries = 5;
  public static final boolean useMotionMagic = false;
  public static final double ampPreset = 5; // tune pls
  public static final double trapPreset = 12; // we never getting this are we
  public static final double stowPreset = 0;

  public static final class sim {
    public static final double simGearing = 30 / 11;
    public static final double carriageMass = 4.420;
    public static final double drumRadius = Units.inchesToMeters(0.47);
    public static final double minHeight = 0.2;
    public static final double maxHeight =
        3; // this isn't right but i also dont care cuz sim is just for fun
    public static final double startingHeight = 0;
  }
}
