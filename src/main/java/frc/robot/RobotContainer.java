// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.swerve.AzimuthConstants.*;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.BeamBreakIOAdafruit;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ampevator.Ampevator;
import frc.robot.subsystems.ampevator.AmpevatorIOSim;
import frc.robot.subsystems.ampevator.AmpevatorIOTalonFX;
import frc.robot.subsystems.ampevatorrollers.Roller;
import frc.robot.subsystems.ampevatorrollers.RollerConstants;
import frc.robot.subsystems.ampevatorrollers.RollerIOTalonFX;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.pivotshooter.PivotShooter;
import frc.robot.subsystems.pivotshooter.PivotShooterIOSim;
import frc.robot.subsystems.pivotshooter.PivotShooterIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindex.ShooterFeederIOTalonFX;
import frc.robot.subsystems.spindex.Spindex;
import frc.robot.subsystems.spindex.SpindexConstants;
import frc.robot.subsystems.spindex.SpindexIOTalonFX;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.MappedXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final CommandSwerveDrivetrain swerve = TunerConstants.DriveTrain;
  private boolean isRed;
  private Trigger rumbleTrigger;

  private final Ampevator ampevator =
      new Ampevator(
          Constants.FeatureFlags.kAmpevatorEnabled,
          (Utils.isSimulation()) ? new AmpevatorIOSim() : new AmpevatorIOTalonFX());

  private final Turret turret =
      new Turret(
          Constants.FeatureFlags.kTurretEnabled,
          new TurretIOTalonFX(),
          new EncoderIOCancoder(TurretConstants.kCanCoderID1),
          new EncoderIOCancoder(TurretConstants.kCanCoderID2));
  private final Shooter shooter =
      new Shooter(
          Constants.FeatureFlags.kShooterEnabled,
          Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOTalonFX());

  private final PivotShooter pivotShooter =
      new PivotShooter(
          Constants.FeatureFlags.kPivotShooterEnabled,
          Utils.isSimulation() ? new PivotShooterIOSim() : new PivotShooterIOTalonFX());

  private final Roller ampevatorRollers =
      new Roller(
          Constants.FeatureFlags.kAmpevatorRollersEnabled,
          new RollerIOTalonFX(),
          new BeamBreakIOAdafruit(RollerConstants.kRollerBeamBreakDIO));

  private final Climb climb = new Climb(Constants.FeatureFlags.kClimbEnabled, new ClimbIOTalonFX());
  private final Intake intake =
      new Intake(
          Constants.FeatureFlags.kIntakeEnabled,
          new IntakeIOTalonFX());
  private final Spindex spindex =
      new Spindex(
          Constants.FeatureFlags.kSpindexEnabled,
          new SpindexIOTalonFX(),
          new ShooterFeederIOTalonFX(),
          new BeamBreakIOAdafruit(SpindexConstants.kSpindexBeamBreakDIO));
  private final Vision vision = new Vision(new VisionIOLimelight());

//  private final Superstructure superstructure =
//      new Superstructure(
//          ampevator,
//          ampevatorRollers,
//          turret,
//          climb,
//          intake,
//          spindex,
//          pivotShooter,
//          shooter,
//          vision);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final MappedXboxController m_driverController =
      new MappedXboxController(ControllerConstants.kDriverControllerPort, "driver");
  public final MappedXboxController m_operatorController =
      new MappedXboxController(ControllerConstants.kOperatorControllerPort, "operator");

  private final AutoRoutines autoRoutines =
      new AutoRoutines(swerve, pivotShooter, intake, shooter, spindex, turret);

  private final AutoChooser autoChooser = new AutoChooser(swerve.autoFactory, "Auto Chooser");
  private final SwerveTelemetry swerveTelemetry =
      new SwerveTelemetry(TunerConstants.kSpeedAt12VoltsMps);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutoChooser();
    configureSwerve();
    configureRumble();

    if (Utils.isSimulation()) {
      SimMechs.init();
    }
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
    m_operatorController
        .a("Intake").onTrue(intake.intakeIn(spindex.debouncedBeamBreak))
            .onTrue(spindex.goToShooter())
            .onTrue(turret.setPosition(TurretConstants.kIntakePreset));
    m_operatorController.b("home").onTrue(turret.setPosition(TurretConstants.kSubPreset));
    m_operatorController.y("shoooot]erae").onTrue(
                    shooter.setVelocity(
                            ShooterConstants.kShooterSpeakerRPS, ShooterConstants.kShooterFollowerSpeakerRPS))
            .onTrue(spindex.feedNoteToShooter());
    m_operatorController
        .rightTrigger("Shooter")
        .onTrue(
            shooter.setVelocity(
                ShooterConstants.kShooterSpeakerRPS, ShooterConstants.kShooterFollowerSpeakerRPS));
  }

  private void configureRumble() {
    rumbleTrigger = new Trigger(RobotModeTriggers.teleop()).and(spindex.debouncedBeamBreak);
    rumbleTrigger
        .onTrue(
            Commands.run(
                () -> {
                  m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                  m_operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                }))
        .onFalse(
            Commands.run(
                () -> {
                  m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                  m_operatorController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 50);
                }));
  }

  private void configureAutoChooser() {
    autoChooser.addAutoRoutine("Box", autoRoutines::boxAuto);
    autoChooser.addAutoRoutine("Source Leave + Preload", autoRoutines::sourceLeave);
    autoChooser.addAutoRoutine("Amp Leave + Preload", autoRoutines::ampLeave);
  }

  public void setAllianceCol(boolean red) {
    isRed = red;
  }

  public void configureSwerve() {
    double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    double MaxAngularRate = 1.5 * Math.PI; // My drivetrain
    double SlowMaxSpeed = MaxSpeed * 0.3;
    double SlowMaxAngular = MaxAngularRate * 0.3;

    SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(Constants.ControllerConstants.DriverConstants.kStickDeadband * MaxSpeed)
            .withRotationalDeadband(
                Constants.ControllerConstants.DriverConstants.kRotationalDeadband
                    * MaxAngularRate) // Add a 10% deadband
            .withDriveRequestType(
                SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
    /* Default swerve command */
    swerve.setDefaultCommand(
        /* Drivetrain will execute this command periodically */
        swerve.applyRequest(
            () ->
                drive
                    .withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive -y is forward
                    .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive -x is left
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));

    /* invert axes */
    m_driverController
        .rightTrigger("Drive with Inverted Axes")
        .whileTrue(
            swerve.applyRequest(
                () ->
                    drive
                        .withVelocityX(-m_driverController.getLeftY())
                        .withVelocityY(-m_driverController.getLeftX())
                        .withRotationalRate(-m_driverController.getRightX())));

    /* translational and rotational speed are slowed */
    m_driverController
        .leftTrigger("Slow Drive")
        .whileTrue(
            swerve.applyRequest(
                () ->
                    drive
                        .withVelocityX(m_driverController.getLeftY() * SlowMaxSpeed)
                        .withVelocityY(m_driverController.getLeftX() * SlowMaxSpeed)
                        .withRotationalRate(-m_driverController.getRightX() * SlowMaxAngular)));

    /* Reset robot heading to current orientation on button press */
    m_driverController.y("reset heading").onTrue(swerve.runOnce(swerve::seedFieldRelative));

    /*
     * Azimuth angle bindings. isRed == true for red alliance presets.
     * isRed != true for blue.
     */
    if (isRed) {
      // azi source red
      m_driverController
          .rightBumper()
          .whileTrue(
              swerve.applyRequest(
                  () ->
                      drive
                          .withVelocityX(m_driverController.getLeftY())
                          .withVelocityY(m_driverController.getLeftX())
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  swerve.getPigeon2().getAngle(),
                                  aziSourceRed.getDegrees(),
                                  Timer.getFPGATimestamp()))));

      // azi amp red
      m_driverController
          .a()
          .whileTrue(
              swerve.applyRequest(
                  () ->
                      drive
                          .withVelocityX(m_driverController.getLeftY())
                          .withVelocityY(m_driverController.getLeftX())
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  swerve.getPigeon2().getAngle(),
                                  aziAmpRed.getDegrees(),
                                  Timer.getFPGATimestamp()))));

      // azi feeder red
      m_driverController
          .povRight()
          .whileTrue(
              swerve.applyRequest(
                  () ->
                      drive
                          .withVelocityX(m_driverController.getLeftY())
                          .withVelocityY(m_driverController.getLeftX())
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  swerve.getPigeon2().getAngle(),
                                  aziFeederRed.getDegrees(),
                                  Timer.getFPGATimestamp()))));
    } else {
      // azi source blue
      m_driverController
          .rightBumper()
          .whileTrue(
              swerve.applyRequest(
                  () ->
                      drive
                          .withVelocityX(m_driverController.getLeftY())
                          .withVelocityY(m_driverController.getLeftX())
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  swerve.getPigeon2().getAngle(),
                                  aziSourceBlue.getDegrees(),
                                  Timer.getFPGATimestamp()))));

      // azi amp blue
      m_driverController
          .a()
          .whileTrue(
              swerve.applyRequest(
                  () ->
                      drive
                          .withVelocityX(m_driverController.getLeftY())
                          .withVelocityY(m_driverController.getLeftX())
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  swerve.getPigeon2().getAngle(),
                                  aziAmpBlue.getDegrees(),
                                  Timer.getFPGATimestamp()))));

      // azi feeder blue
      m_driverController
          .povRight()
          .whileTrue(
              swerve.applyRequest(
                  () ->
                      drive
                          .withVelocityX(m_driverController.getLeftY())
                          .withVelocityY(m_driverController.getLeftX())
                          .withRotationalRate(
                              SwerveConstants.azimuthController.calculate(
                                  swerve.getPigeon2().getAngle(),
                                  aziFeederBlue.getDegrees(),
                                  Timer.getFPGATimestamp()))));
    }

    /* Universal Azimuth Bindings */

    // azi subwoofer front
    m_driverController
        .leftBumper()
        .whileTrue(
            swerve.applyRequest(
                () ->
                    drive
                        .withVelocityX(m_driverController.getLeftY())
                        .withVelocityY(m_driverController.getLeftX())
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                swerve.getPigeon2().getAngle(),
                                aziSubwooferFront.getDegrees(),
                                Timer.getFPGATimestamp()))));

    // azi subwoofer left
    m_driverController
        .x()
        .whileTrue(
            swerve.applyRequest(
                () ->
                    drive
                        .withVelocityX(m_driverController.getLeftY())
                        .withVelocityY(m_driverController.getLeftX())
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                swerve.getPigeon2().getAngle(),
                                aziSubwooferLeft.getDegrees(),
                                Timer.getFPGATimestamp()))));

    // azi subwoofer right
    m_driverController
        .b()
        .whileTrue(
            swerve.applyRequest(
                () ->
                    drive
                        .withVelocityX(m_driverController.getLeftY())
                        .withVelocityY(m_driverController.getLeftX())
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                swerve.getPigeon2().getAngle(),
                                aziSubwooferRight.getDegrees(),
                                Timer.getFPGATimestamp()))));

    // azi cleanup
    m_driverController
        .povDown()
        .whileTrue(
            swerve.applyRequest(
                () ->
                    drive
                        .withVelocityX(m_driverController.getLeftY())
                        .withVelocityY(m_driverController.getLeftX())
                        .withRotationalRate(
                            SwerveConstants.azimuthController.calculate(
                                swerve.getPigeon2().getAngle(),
                                aziCleanUp.getDegrees(),
                                Timer.getFPGATimestamp()))));

    if (Utils.isSimulation()) {
      swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    swerve.registerTelemetry(swerveTelemetry::telemeterize);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return shooter.setVelocity(90, 90);
    return autoChooser.getSelectedAutoRoutine();
  }

  public void periodic() {
    autoChooser.update();
//    superstructure.periodic();
  }
}
