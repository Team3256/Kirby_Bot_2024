// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.ampevator.Ampevator;
import frc.robot.subsystems.ampevator.AmpevatorIOSim;
import frc.robot.subsystems.ampevator.AmpevatorIOTalonFX;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterIOSim;
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveTelemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.turret.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;

  private final Ampevator ampevator =
      new Ampevator(true, (Utils.isSimulation()) ? new AmpevatorIOSim() : new AmpevatorIOTalonFX());

  private final Turret turret =
      new Turret(
          Constants.FeatureFlags.kTurretEnabled,
          new TurretIOTalonFX(),
          new EncoderIOCancoder(TurretConstants.kCanCoderID1),
          new EncoderIOCancoder(TurretConstants.kCanCoderID2));
  private final Shooter shooter =
      new Shooter(
          Constants.FeatureFlags.kShooterEnabled,
          RobotBase.isReal() ? new ShooterIOTalonFX() : new ShooterIOSim());

  private final PivotShooter pivotShooter =
      new PivotShooter(
          Constants.FeatureFlags.kPivotShooterEnabled,
          Utils.isSimulation() ? new PivotShooterIOSim() : new PivotShooterIOTalonFX());

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  private final AutoRoutines autoRoutines = new AutoRoutines(swerve);

  private final AutoChooser autoChooser = new AutoChooser(swerve.autoFactory, "Auto Chooser");
  private final SwerveTelemetry swerveTelemetry =
      new SwerveTelemetry(TunerConstants.kSpeedAt12VoltsMps);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoChooser();
    if (Utils.isSimulation()) {
      swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    swerve.registerTelemetry(swerveTelemetry::telemeterize);

    SimMechs.init();
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

    m_driverController.b().whileTrue(shooter.setVelocity(100, 100));
    m_driverController.x().onTrue(pivotShooter.setPosition(100));
    m_driverController.y().onTrue(pivotShooter.setPosition(-100));
  }

  private void configureAutoChooser() {
    autoChooser.addAutoRoutine("Box", autoRoutines::boxAuto);
  }

  private void configureSwerve() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelectedAutoRoutine();
  }

  public void periodic() {
    autoChooser.update();
  }
}
