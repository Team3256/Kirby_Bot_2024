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
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ampevator.Ampevator;
import frc.robot.subsystems.ampevator.AmpevatorIOSim;
import frc.robot.subsystems.ampevator.AmpevatorIOTalonFX;
import frc.robot.subsystems.ampevatorrollers.Roller;
import frc.robot.subsystems.ampevatorrollers.RollerIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterIOSim;
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindex.BeamBreakIOBanner;
import frc.robot.subsystems.spindex.Spindex;
import frc.robot.subsystems.spindex.SpindexIOTalonFX;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveTelemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.ControllerMapper;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;

  private final Ampevator ampevator = new Ampevator(true,
      (Utils.isSimulation()) ? new AmpevatorIOSim() : new AmpevatorIOTalonFX());

  private final Turret turret = new Turret(
      Constants.FeatureFlags.kTurretEnabled,
      new TurretIOTalonFX(),
      new EncoderIOCancoder(TurretConstants.kCanCoderID1),
      new EncoderIOCancoder(TurretConstants.kCanCoderID2));
  private final Shooter shooter = new Shooter(
      Constants.FeatureFlags.kShooterEnabled,
      RobotBase.isReal() ? new ShooterIOTalonFX() : new ShooterIOSim());

  private final PivotShooter pivotShooter = new PivotShooter(
      Constants.FeatureFlags.kPivotShooterEnabled,
      Utils.isSimulation() ? new PivotShooterIOSim() : new PivotShooterIOTalonFX());

  private final Roller ampevatorRollers = new Roller(Constants.FeatureFlags.kAmpevatorRollersEnabled,
      new RollerIOTalonFX());

  private final Climb climb = new Climb(Constants.FeatureFlags.kClimbEnabled, new ClimbIOTalonFX());
  private final Intake intake = new Intake(Constants.FeatureFlags.kIntakeEnabled, new IntakeIOTalonFX());
  private final Spindex spindex = new Spindex(
      Constants.FeatureFlags.kSpindexEnabled, new SpindexIOTalonFX(), new BeamBreakIOBanner());
  private final Vision vision = new Vision(new VisionIOLimelight());

  private final Superstructure superstructure = new Superstructure(
      ampevator,
      ampevatorRollers,
      turret,
      climb,
      intake,
      spindex,
      pivotShooter,
      shooter,
      vision);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      ControllerConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      ControllerConstants.kOperatorControllerPort);

  private final ControllerMapper controls = new ControllerMapper(m_driverController, m_operatorController);

  private final AutoRoutines autoRoutines = new AutoRoutines(swerve);

  private final AutoChooser autoChooser = new AutoChooser(swerve.autoFactory, "Auto Chooser");
  private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(TunerConstants.kSpeedAt12VoltsMps);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controls.bindDriver("b", "Rev Shooter").whileTrue(shooter.setVelocity(100, 100));
    controls.bindDriver("x", "Set pivot shooter position 100").onTrue(pivotShooter.setPosition(100));
    controls.bindDriver("y", "Set pivot shooter position -100").onTrue(pivotShooter.setPosition(-100));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // This should be at the end of the configureBindings method.
    // No other bindings should be added after this line.
    if (Constants.FeatureFlags.kControllerMapEnabled) {
      controls.dumpControllerMap();
    }
  }

  private void configureAutoChooser() {
    autoChooser.addAutoRoutine("Box", autoRoutines::boxAuto);
  }

  private void configureSwerve() {
  }

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
