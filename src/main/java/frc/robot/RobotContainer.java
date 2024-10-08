// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.AutoRoutines;
import frc.robot.sim.SimMechs;
import frc.robot.subsystems.BeamBreakIOBanner;
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
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.spindex.Spindex;
import frc.robot.subsystems.spindex.SpindexConstants;
import frc.robot.subsystems.spindex.SpindexIOTalonFX;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.requests.SwerveFieldCentricFacingAngle;
import frc.robot.subsystems.turret.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.utils.MappedXboxController;

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

    private final Roller ampevatorRollers = new Roller(
            Constants.FeatureFlags.kAmpevatorRollersEnabled,
            new RollerIOTalonFX(),
            new BeamBreakIOBanner(RollerConstants.kRollerBeamBreakDIO));

    private final Climb climb = new Climb(Constants.FeatureFlags.kClimbEnabled, new ClimbIOTalonFX());
    private final Intake intake = new Intake(
            Constants.FeatureFlags.kIntakeEnabled,
            new IntakeIOTalonFX(),
            new BeamBreakIOBanner(IntakeConstants.kIntakeBeamBreakDIO));
    private final Spindex spindex = new Spindex(
            Constants.FeatureFlags.kSpindexEnabled,
            new SpindexIOTalonFX(),
            new BeamBreakIOBanner(SpindexConstants.kSpindexBeamBreakDIO));
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
    private final MappedXboxController m_driverController = new MappedXboxController(
            ControllerConstants.kDriverControllerPort, "driver");
    private final MappedXboxController m_operatorController = new MappedXboxController(
            ControllerConstants.kOperatorControllerPort, "operator");

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
        configureSwerve();

        SimMechs.init();

        // No other bindings should be added after this line.
        if (Constants.FeatureFlags.kControllerMapEnabled) {
            MappedXboxController.dumpControllerMap(m_driverController, m_operatorController);
        }
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
        m_driverController.b("shoot").whileTrue(shooter.setVelocity(100, 100));
        m_driverController.x("pivot shooter wow").onTrue(pivotShooter.setPosition(100));
        m_driverController.y("pivot shooter wow 2").onTrue(pivotShooter.setPosition(-100));
    }

    private void configureAutoChooser() {
        autoChooser.addAutoRoutine("Box", autoRoutines::boxAuto);
    }

    private void configureSwerve() {
        double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        double MaxAngularRate = 1.5 * Math.PI; // My drivetrain
        double SlowMaxSpeed = MaxSpeed * 0.3;
        double SlowMaxAngular = MaxAngularRate * 0.3;
        SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.ControllerConstants.DriverConstants.kStickDeadband * MaxSpeed)
                .withRotationalDeadband(
                        Constants.ControllerConstants.DriverConstants.kRotationalDeadband
                                * MaxAngularRate) // Add a 10% deadband
                .withDriveRequestType(
                        SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
        SwerveFieldCentricFacingAngle azi = new SwerveFieldCentricFacingAngle()
                .withDeadband(MaxSpeed * .15) // TODO: update deadband
                .withRotationalDeadband(MaxAngularRate * .15) // TODO: update deadband
                .withHeadingController(SwerveConstants.azimuthController)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
        swerve.setDefaultCommand(
                // Drivetrain will execute this command periodically
                swerve.applyRequest(
                        () -> drive
                                .withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive -y is forward
                                .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive -x is left
                                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));
        /*
         * Right stick absolute angle mode on trigger hold,
         * robot adjusts heading to the angle right joystick creates
         */
        m_driverController
                .rightTrigger("something")
                .whileTrue(
                        swerve.applyRequest(
                                () -> drive
                                        .withVelocityX(
                                                -m_driverController.getLeftY() * MaxSpeed) // Drive -y is forward
                                        .withVelocityY(
                                                -m_driverController.getLeftX() * MaxSpeed) // Drive -x is left
                                        .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)));
        // Slows translational and rotational speed to 30%
        m_driverController
                .leftTrigger("something")
                .whileTrue(
                        swerve.applyRequest(
                                () -> drive
                                        .withVelocityX(m_driverController.getLeftY() * (MaxSpeed * 0.17))
                                        .withVelocityY(m_driverController.getLeftX() * (MaxSpeed * 0.17))
                                        .withRotationalRate(
                                                -m_driverController.getRightX() * (1.3 * 0.2 * Math.PI))));
        // Reset robot heading on button press
        m_driverController.y("seed relative").onTrue(swerve.runOnce(() -> swerve.seedFieldRelative()));
        // Azimuth angle bindings. isRed == true for red alliance presets. isRed != true
        // for blue.
        if (DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Red)) {
            m_driverController
                    .rightBumper("smth")
                    .whileTrue(
                            swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziSourceRed)));
            m_driverController
                    .a("smth")
                    .whileTrue(
                            swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziAmpRed)));
            m_driverController
                    .povRight("smth")
                    .whileTrue(
                            swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziFeederRed)));
        } else {
            m_driverController
                    .rightBumper("smth")
                    .whileTrue(
                            swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziSourceBlue)));
            m_driverController
                    .a("smth")
                    .whileTrue(
                            swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziAmpBlue)));
            m_driverController
                    .povRight("smth")
                    .whileTrue(
                            swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziFeederBlue)));
        }
        // Universal azimuth bindings
        m_driverController
                .leftBumper("smth")
                .whileTrue(
                        swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziSubwooferFront)));
        m_driverController
                .povDownLeft("azi")
                .whileTrue(
                        swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziSubwooferLeft)));
        m_driverController
                .povDownRight("azi")
                .whileTrue(
                        swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziSubwooferRight)));
        m_driverController
                .povDown("azi")
                .whileTrue(swerve.applyRequest(() -> azi.withTargetDirection(AzimuthConstants.aziCleanUp)));
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
    }
}
