// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class ControllerMapper {
  private CommandXboxController driver;
  private CommandXboxController operator;
  private boolean enabled;

  private Map<String, Map<String, String>> buttonMap = new HashMap<>();

  public ControllerMapper(
      boolean enabled, CommandXboxController driver, CommandXboxController operator) {
    this.enabled = enabled;
    this.driver = driver;
    this.operator = operator;
    buttonMap.put("driver", new HashMap<>());
    buttonMap.put("operator", new HashMap<>());
  }

  public Trigger bind(String controller, String button, String commmandDescription) {
    if (controller.equals("driver")) {
      return getButton(driver, button);
    } else {
      return getButton(operator, button);
    }
  }

  public Trigger bindOperator(String button, String commmandDescription) {
    buttonMap.get("operator").put(button, commmandDescription);
    return getButton(operator, button);
  }

  public Trigger bindDriver(String button, String commmandDescription) {
    buttonMap.get("driver").put(button, commmandDescription);
    return getButton(driver, button);
  }

  private Trigger getButton(CommandXboxController controller, String button) {
    switch (button) {
      case "a":
        return controller.a();
      case "b":
        return controller.b();
      case "x":
        return controller.x();
      case "y":
        return controller.y();
      case "leftBumper":
        return controller.leftBumper();
      case "rightBumper":
        return controller.rightBumper();
      case "start":
        return controller.start();
      case "back":
        return controller.back();
      case "leftStick":
        return controller.leftStick();
      case "rightStick":
        return controller.rightStick();
      case "leftTrigger":
        return controller.leftTrigger();
      case "rightTrigger":
        return controller.rightTrigger();
      default:
        return null;
    }
  }

  public void outputControllerMap() {
    if (!this.enabled) {
      return;
    }
    Gson gson = new Gson();
    Logger.recordOutput("controllerMap", gson.toJson(buttonMap));
  }

  public void setLeftJoystickDescription(String controller, String description) {}

  public void setRightJoystickDescription(String controller, String description) {}
}
