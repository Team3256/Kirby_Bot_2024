// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.LimelightHelpers;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private final boolean enabled;
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final PIDController xController = new PIDController(10, 0, 0);
  private final PIDController yController = new PIDController(10, 0, 0);
  private final PIDController thetaController = new PIDController(7, 0, 0);

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  public final AutoFactory autoFactory =
      Choreo.createAutoFactory(
          this,
          () -> this.getState().Pose,
          this::choreoController,
          (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
          () ->
              DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                  == DriverStation.Alliance.Red,
          new AutoFactory.ChoreoAutoBindings());

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

  public CommandSwerveDrivetrain(
      boolean enabled,
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    this.enabled = enabled;
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      boolean enabled,
      SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    this.enabled = enabled;
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  @Override
  public void setControl(SwerveRequest request) {
    if (enabled) {
      super.setControl(request);
    }
  }

  public ChassisSpeeds choreoController(Pose2d currentPose, SwerveSample sample) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            xController.calculate(currentPose.getX(), sample.x) + sample.vx,
            yController.calculate(currentPose.getY(), sample.y) + sample.vy,
            thetaController.calculate(currentPose.getRotation().getRadians(), sample.heading)
                + sample.omega),
        currentPose.getRotation());
  }

  public void updateVision() {
    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        this.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(
          "limelight",
          this.m_odometry.getEstimatedPosition().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (Math.abs(this.getPigeon2().getRate())
          > 720) // if our angular velocity is greater than 720 degrees per
      // second, ignore vision
      // updates
      {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        this.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
      }
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            SwerveConstants.autoTranslationalController,
            SwerveConstants.autoRotationalController,
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                == DriverStation.Alliance.Red, // Assume
        // the path
        // needs to
        // be
        // flipped
        // for Red
        // vs Blue,
        // this is
        // normally
        // the case
        this); // Subsystem for requirements
  }

  public Command pathfindToNote(Vision vision) {
    PathConstraints constraints =
        new PathConstraints(
            TunerConstants.kSpeedAt12VoltsMps - 1, // max speed (4.96)
            4, // max acceleration (4 m/s^2)
            edu.wpi.first.math.util.Units.degreesToRadians(450), // max angular velocity (450 deg/s)
            edu.wpi.first.math.util.Units.degreesToRadians(
                540)); // max angular acceleration (540 deg/s^2)
    return AutoBuilder.pathfindToPose(
        // current pose, path constraints (see above), "goal end velocity", rotation
        // delay distance (how long to travel before rotating)
        vision.getNotePose(this.getState().Pose), constraints, 1, 0.0);
  }

  public Command rotationTest() {
    PathConstraints constraints =
        new PathConstraints(
            TunerConstants.kSpeedAt12VoltsMps - 1, // max speed (4.96)
            4, // max acceleration (4 m/s^2)
            edu.wpi.first.math.util.Units.degreesToRadians(450), // max angular velocity (450 deg/s)
            edu.wpi.first.math.util.Units.degreesToRadians(
                540)); // max angular acceleration (540 deg/s^2)
    return AutoBuilder.pathfindToPose(
        // current pose, path constraints (see above), "goal end velocity", rotation
        // delay distance (how long to travel before rotating)
        this.getState()
            .Pose
            .rotateBy(Rotation2d.fromDegrees(90))
            .plus(new Transform2d(new Translation2d(2, 1), Rotation2d.fromDegrees(0))),
        constraints,
        0,
        0.0);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void resetOdometry(AutoTrajectory traj) {
    this.m_odometry.resetPosition(
        traj.getInitialPose().get().getRotation(), m_modulePositions, traj.getInitialPose().get());
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  // public Command runChoreoTraj(ChoreoTrajectory trajectory) {
  // return Choreo.choreoSwerveCommand(
  // trajectory,
  // () -> (this.getState().Pose),
  // SwerveConstants.choreoTranslationController,
  // SwerveConstants.choreoTranslationController,
  // SwerveConstants.choreoRotationController,
  // ((ChassisSpeeds speeds) ->
  // this.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds))),
  // () -> {
  // Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
  // return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  // },
  // this);
  // }

  /*
   * This method comes from 1690's <a
   * href="https://www.youtube.com/watch?v=N6ogT5DjGOk&t=1674s">Second Software
   * Presentation</a>
   * we get the skidding ratio from the current SwerveModuleStates
   * The skidding ratio is defined as the ratio between the maximum and the
   * minimum magnitudes of the "translational" part of the velocity vector of the
   * robot.
   *
   * @return the skidding ratio of the robot, maximum/minimum, ranges from
   * [1,INFINITY)
   */
  public double getSkiddingRatio() {
    // josh: accessing swerveStates / kinematics like this is INTENTIONAL.

    // grab the current SwerveModuleStates & SwerveDriveKinematics.
    final SwerveModuleState[] swerveStates = super.getState().ModuleStates;
    final SwerveDriveKinematics kinematics = super.m_kinematics;

    // get the angular velocity of the robot
    final double angularVelocityOmegaMeasured =
        kinematics.toChassisSpeeds(swerveStates).omegaRadiansPerSecond; // use
    // IK to
    // get a
    // chassis
    // speed,
    // then
    // pull
    // out
    // the
    // angular velocity

    // get the rotational SwerveModuleStates (i.e SwerveModuleState with only angle)
    final SwerveModuleState[] swerveStatesRotational =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));

    final double[] swerveStatesTranslationalMagnitudes =
        new double[super.getState().ModuleStates.length];
    // josh: intentionally doing this instead of directly pulling
    // speedMetersPerSecond from the SwerveModuleState
    for (int i = 0; i < swerveStates.length; i++) {
      final Translation2d
          swerveStateTranslation2d = convertSwerveModuleStateToTranslation2d(swerveStates[i]),
          swerveStateRotational =
              convertSwerveModuleStateToTranslation2d(swerveStatesRotational[i]),
          swerveStateTranslational = swerveStateTranslation2d.minus(swerveStateRotational);
      swerveStatesTranslationalMagnitudes[i] = swerveStateTranslational.getNorm();
    }

    // find the maximum and minimum magnitudes of the translational parts of the
    // SwerveModuleStates
    double max = Double.MIN_VALUE, min = Double.MAX_VALUE;
    for (double translationalSpeed : swerveStatesTranslationalMagnitudes) {
      max = Math.max(max, translationalSpeed);
      min = Math.min(min, translationalSpeed);
    }

    // return the skidding ratio
    return max / min;
  }

  /*
   * Translation2d is the wpilib class that represents a 2d vector. this method
   * could really be called
   * "convertSwerveModuleStateToVector"
   */
  private Translation2d convertSwerveModuleStateToTranslation2d(SwerveModuleState state) {
    return new Translation2d(state.speedMetersPerSecond, state.angle);
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    /*
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled
     */
    /*
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == DriverStation.Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
      if (Constants.FeatureFlags.kVisionEnabled) {
        updateVision();
      }
    }
  }
}
