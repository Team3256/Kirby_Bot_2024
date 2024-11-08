// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.PhoenixUtil;

public class EncoderIOCancoder implements EncoderIO {

  private final CANcoder encoder;

  private final StatusSignal<Double> encoderPosition;
  private final StatusSignal<Double> encoderVelocity;

  public EncoderIOCancoder(int canCoderID) {
    encoder = new CANcoder(canCoderID);
    PhoenixUtil.applyCancoderConfig(
        encoder, TurretConstants.canCoderConfig, TurretConstants.flashConfigRetries);

    encoderPosition = encoder.getAbsolutePosition();
    encoderVelocity = encoder.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        TurretConstants.updateFrequency, encoderPosition, encoderVelocity);
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(EncoderIOInputs inputs) {
    BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);
    inputs.encoderPositionDegrees = Units.rotationsToDegrees(encoderPosition.getValueAsDouble());
    inputs.encoderVelocity = encoderVelocity.getValueAsDouble();
  }

  @Override
  public void zero() {
    encoder.setPosition(0.0);
  }
}
