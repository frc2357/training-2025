package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.RobotMath;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final SwerveRequest.ApplyRobotSpeeds robotSpeedRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.FieldCentric fieldRelative =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotRelative =
        new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public void applyRequest(Supplier<SwerveRequest> requestSupplier) {
        setControl(requestSupplier.get());
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public double getYaw() {
        return getPigeon2().getYaw().getValueAsDouble();
    }

    // Pigeon is rotated 90 degrees so pitch and roll are flipped
    public double getRoll() {
        return getPigeon2().getPitch().getValueAsDouble();
    }

    // Pigeon is rotated 90 degrees so pitch and roll are flipped
    public double getPitch() {
        return getPigeon2().getRoll().getValueAsDouble();
    }

    public void zeroGyro(boolean flip) {
        StatusCode code = super.getPigeon2().setYaw(flip ? 180 : 0);
        System.out.println("[GYRO] Zeroed to " + (flip ? 180 : 0) + ": " + code.toString());
    }

    public void stopMotorsIntoX() {
        applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }

    /** Stops the motors in a way that should make them not jingle. */
    public void stopMotors() {
        driveFieldRelative(0, 0, 0);
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : super.getModules()) {
            module.getDriveMotor().stopMotor(); // anti-jingle
            module.getSteerMotor().stopMotor(); // remove to bring back the jingle (dont do it)
        }
    }

    /**
     * Returns the current field relative speeds but in a ChassisSpeeds object.
     *
     * @return the current field relative speeds in a ChassisSpeeds object.
     */
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        ChassisSpeeds chassisSpeeds = getKinematics().toChassisSpeeds(getModuleStates());
        return chassisSpeeds;
    }

    /**
     * @return A list of module positions in the order Front Left, Front Right, Back Left, Back Right
     */
    public Translation2d[] getModulePositions() {
        return super.getModuleLocations();
    }

    /**
     * @return A list of module states in the order Front Left, Front Right, Back Left, Back Right
     */
    public SwerveModuleState[] getModuleStates() {
        return super.getState().ModuleStates;
    }

    /**
     * @return A list of module targets in the order Front Left, Front Right, Back Left, Back Right
     */
    public SwerveModuleState[] getModuleTargets() {
        return super.getState().ModuleTargets;
    }

    public Pose2d getPose() {
        return super.getState().Pose;
    }

    /**
     * The method to use to drive while targetlocking.
     *
     * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
     * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
     * @param yaw The current yaw of the robot, if in tolerance, pass in the setpoint instead.
     * @param yawSetpoint The setpoint to use for the yaw PID controller.
     * @param hasTarget Whether or not there is a target.
     */
    public void driveTargetLock(
        double velocityXSpeedMetersPerSecond,
        double velocityYSpeedMetersPerSecond,
        double yaw,
        double yawSetpoint,
        boolean hasTarget) {
        double vy = getFieldRelativeChassisSpeeds().vyMetersPerSecond; // Horizontal velocity
        double kp = Constants.SWERVE.TARGET_LOCK_ROTATION_KP;
        kp *= Math.max(1, vy * 1);
        Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.setP(kp);

        double rotation =
            Constants.SWERVE.TARGET_LOCK_ROTATION_PID_CONTROLLER.calculate(yaw, yawSetpoint);
        // if we have a target, add feedforward to the controls, if we dont, let the driver rotate the
        // robot manually.
        double rotationOutput =
            !hasTarget
                ? Robot.driverControls.getRotation() * SWERVE.MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND
                : rotation + Math.copySign(Constants.SWERVE.TARGET_LOCK_FEED_FORWARD, rotation);
        driveFieldRelative(
            velocityXSpeedMetersPerSecond, velocityYSpeedMetersPerSecond, rotationOutput);
    }

    /**
     * The method to use for robot relative driving.
     *
     * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
     * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
     * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
     */
    public void driveRobotRelative(
        double velocityXMetersPerSecond,
        double velocityYMetersPerSecond,
        double rotationRateRadiansPerSecond) {
        applyRequest(
            () ->
                robotRelative
                    .withVelocityX(velocityXMetersPerSecond)
                    .withVelocityY(velocityYMetersPerSecond)
                    .withRotationalRate(rotationRateRadiansPerSecond));
    }

    /**
     * The method to use for field relative driving.
     *
     * @param velocityXSpeedMetersPerSecond The desired speed on the X axis in meters per second.
     * @param velocityYSpeedMetersPerSecond The desired speed on the X axis in meters per second.
     * @param rotationRateRadiansPerSecond The desired rotation rate in radians per second.
     */
    public void driveFieldRelative(
        double velocityXMetersPerSecond,
        double velocityYMetersPerSecond,
        double rotationRateRadiansPerSecond) {
        applyRequest(
            () ->
                fieldRelative
                    .withVelocityX(velocityXMetersPerSecond)
                    .withVelocityY(velocityYMetersPerSecond)
                    .withRotationalRate(rotationRateRadiansPerSecond));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Gets the yaw to turn the robot to for target locking so we shoot into the speaker.
     *
     * @param pitch The current pitch of the middle speaker april tag.
     * @param defualtOffset The default offset to return if we dont have a valid curve index.
     * @return The yaw to turn the robot to for target locking.
     */
    public double updateVisionTargeting(double pitch, double defaultOffset) {
        int curveIndex = RobotMath.getCurveSegmentIndex(Robot.shooterCurve, pitch);
        if (curveIndex == -1) {
        return defaultOffset;
        }

        double[] high = Robot.shooterCurve[curveIndex];
        double[] low = Robot.shooterCurve[curveIndex + 1];

        double highPitch = high[0];
        double lowPitch = low[0];
        double highYawSetopint = high[3];
        double lowYawSetpoint = low[3];

        return RobotMath.linearlyInterpolate(
            highYawSetopint, lowYawSetpoint, highPitch, lowPitch, pitch);
    }
}
