// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.turret.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Turret turret =
      new Turret(
          Constants.FeatureFlags.kTurretEnabled,
          new TurretIOTalonFX(),
          new EncoderIOCancoder(TurretConstants.kCanCoderID1),
          new EncoderIOCancoder(TurretConstants.kCanCoderID2));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    bind("driver", "b", "exampleMethodCommand")
        .whileTrue(m_exampleSubsystem.exampleMethodCommand());
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

  private Trigger bind(String controller, String button, String commmandDescription) {
    if (controller.equals("driver")) {
      return getButton(m_driverController, button);
    } else {
      return getButton(m_operatorController, button);
    }
  }

  private void setLeftJoystickDescription(String controller, String description) {}

  private void setRightJoystickDescription(String controller, String description) {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
