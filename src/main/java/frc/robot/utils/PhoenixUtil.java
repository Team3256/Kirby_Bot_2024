// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Supplier;

// The only method you should really use is applyMotorConfigs
public class PhoenixUtil {
  // Spams fetching StatusCode until it works or we run out of attempts
  public static boolean spamGetStatusCode(Supplier<StatusCode> function, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      DriverStation.reportError(
          "Failed to execute phoenix pro api call after " + numTries + " attempts", false);
      return false;
    }
    return true;
  }

  public static boolean spamGetStatusCode(Supplier<StatusCode> function) {
    // Default that 254 uses
    return spamGetStatusCode(function, 5);
  }

  public static boolean readAndVerifyConfiguration(TalonFX talon, TalonFXConfiguration config) {
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    if (!spamGetStatusCode(() -> talon.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
          "Failed to read config for talon [" + talon.getDescription() + "]", false);
      return false;
    } else if (!TalonConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
          "Configuration verification failed for talon [" + talon.getDescription() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static boolean readAndVerifyConfiguration(
      CANcoder cancoder, CANcoderConfiguration config) {
    CANcoderConfiguration readConfig = new CANcoderConfiguration();
    if (!spamGetStatusCode(() -> cancoder.getConfigurator().refresh(readConfig))) {
      // could not get config!
      DriverStation.reportWarning(
          "Failed to read config for CANCoder [" + cancoder.getDeviceID() + "]", false);
      return false;
    } else if (!TalonConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      DriverStation.reportWarning(
          "Configuration verification failed for cancoder [" + cancoder.getDeviceID() + "]", false);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  // The main function you should use for most purposes
  public static boolean applyMotorConfigs(
      TalonFX motor, TalonFXConfiguration motorConfig, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (RobotBase.isSimulation()) {
        return motor.getConfigurator().apply(motorConfig).isOK();
      }
      if (spamGetStatusCode(() -> motor.getConfigurator().apply(motorConfig))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(motor, motorConfig)) {
          return true;
        } else {
          DriverStation.reportWarning(
              "Failed to verify config for talon ["
                  + motor.getDescription()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")",
              false);
        }
      } else {
        DriverStation.reportWarning(
            "Failed to apply config for talon ["
                + motor.getDescription()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")",
            false);
      }
    }
    DriverStation.reportError(
        "Failed to apply config for talon after " + numTries + " attempts", false);
    return false;
  }

  // TODO(Bryan): in the future I want to make this generic LOL
  // but right now it's not worth the effort
  public static boolean applyCancoderConfig(
      CANcoder cancoder, CANcoderConfiguration cancoderConfig, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (RobotBase.isSimulation()) {
        return cancoder.getConfigurator().apply(cancoderConfig).isOK();
      }
      if (spamGetStatusCode(() -> cancoder.getConfigurator().apply(cancoderConfig))) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(cancoder, cancoderConfig)) {
          return true;
        } else {
          DriverStation.reportWarning(
              "Failed to verify config for cancoder ["
                  + cancoder.getDeviceID()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")",
              false);
        }
      } else {
        DriverStation.reportWarning(
            "Failed to apply config for cancoder ["
                + cancoder.getDeviceID()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")",
            false);
      }
    }
    DriverStation.reportError(
        "Failed to apply config for talon after " + numTries + " attempts", false);
    return false;
  }
}
