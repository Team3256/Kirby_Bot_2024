// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.NT4PublisherNoFMS;
import monologue.Logged;
import monologue.Monologue;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot implements Logged {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // AdvantageKit >>
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4PublisherNoFMS()); // Publish data to NetworkTables
      // Enables power distribution logging
      new PowerDistribution(
          1, ModuleType.kRev); // Ignore this "resource leak"; it was the example code from docs
    } else {
      setUseTiming(false); // Run as fast as possible
      // String logPath =
      // LogFileUtil
      // .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the
      // user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(
      // new WPILOGWriter(
      // LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    if (Constants.FeatureFlags.kAdvKitEnabled) {
      // Start logging! No more data receivers, replay sources, or metadata values may
      // be added.
      Logger.start();
    }
    // << AdvantageKit
    // Monologue >>
    if (Constants.FeatureFlags.kMonologueEnabled) {
      Monologue.setupMonologue(
          this, "MonologueRobot", Constants.kMonologueFileOnly, Constants.kMonologueFileOnly);
    }
    // << Monologue
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Monologue >>
    if (Constants.FeatureFlags.kMonologueEnabled) {
      // setFileOnly is used to shut off NetworkTables broadcasting for most
      // logging calls.
      // Basing this condition on the connected state of the FMS is a suggestion only.
      Monologue.setFileOnly(DriverStation.isFMSAttached());
      // This method needs to be called periodically, or no logging annotations will
      // process
      // properly.
      Monologue.updateAll();
    }
    // << Monologue
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
