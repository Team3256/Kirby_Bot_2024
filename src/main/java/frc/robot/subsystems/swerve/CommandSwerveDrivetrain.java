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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private final PIDController xController = new PIDController(5, 0, 0);
  private final PIDController yController = new PIDController(5, 0, 0);
  private final PIDController thetaController = new PIDController(5, 0, 0);

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

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void resetOdometry(AutoTrajectory traj) {
    this.m_odometry.resetPosition(
        traj.getInitialPose().get().getRotation(), m_modulePositions, traj.getInitialPose().get());
  }

  public Command resetOdometryCmd(AutoTrajectory traj) {
    return Commands.runOnce(() -> resetOdometry(traj));
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
    }
  }
}
