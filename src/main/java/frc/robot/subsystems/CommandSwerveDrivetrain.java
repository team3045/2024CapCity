package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static final PIDController HEADING_CONTROLLER = new PIDController(10, 0, 0);
    public static final double rangeTheshold = 3; // 3 meters

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(TranslationCharacterization.withVolts(volts)),
                    null,
                    this));

    @SuppressWarnings("unused")
    private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(RotationCharacterization.withVolts(volts)),
                    null,
                    this));
    @SuppressWarnings("unused")
    private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setControl(SteerCharacterization.withVolts(volts)),
                    null,
                    this));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

    /* Publishing */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable armTable = inst.getTable("DrivePose");
    private final StructPublisher<Pose2d> pose2dPublisher = armTable.getStructTopic("Drive Pose2d", Pose2d.struct)
            .publish();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
            seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
            seedFieldRelative(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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
                (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be
                                                                                         // flipped for Red vs Blue,
                                                                                         // this is normally the case
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
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

    public Command aimAtSpeakerMoving(DoubleSupplier vX, DoubleSupplier vY) {
        return driveFacingAngleCommand(vX, vY, () -> getSpeakerAimingPoint());
    }

    public Rotation2d getSpeakerAimingPoint() {
        Pose2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.targetPoseBlue
                : FieldConstants.targetPoseRed;

        Pose2d robotPose = getState().Pose;

        Translation2d fieldRobotSpeeds = new Translation2d(
                getState().speeds.vxMetersPerSecond,
                getState().speeds.vyMetersPerSecond);

        /* Predict where target will be based on our current speeds */
        Translation2d virtualTarget = target.getTranslation()
                .plus(
                        fieldRobotSpeeds.times(ShooterConstants.tangentialNoteFlightTime)
                                .rotateBy(Rotation2d.fromDegrees(180.0)));

        Translation2d targetRelativeToRobot = virtualTarget.minus(robotPose.getTranslation());

        return new Rotation2d(targetRelativeToRobot.getX(), targetRelativeToRobot.getY());
    }

    public double getSpeakerDistanceMoving() {
        Pose2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.targetPoseBlue
                : FieldConstants.targetPoseRed;

        Pose2d robotPose = getState().Pose;

        Translation2d fieldRobotSpeeds = new Translation2d(
                getState().speeds.vxMetersPerSecond,
                getState().speeds.vyMetersPerSecond);

        /* Predict where target will be based on our current speeds */
        Translation2d virtualTarget = target.getTranslation()
                .plus(
                        fieldRobotSpeeds.times(ShooterConstants.tangentialNoteFlightTime)
                                .rotateBy(Rotation2d.fromDegrees(180.0)));

        Translation2d targetRelativeToRobot = virtualTarget.minus(robotPose.getTranslation());
        return targetRelativeToRobot.getNorm();
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command getDriveCommand(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vOmega) {
        return applyRequest(() -> drive.withVelocityX(vX.getAsDouble()) // Drive forward with
                // negative Y (forward)
                .withVelocityY(vY.getAsDouble()) // Drive left with negative X (left)
                .withRotationalRate(vOmega.getAsDouble()));// Drive counterclockwise with negative X (left)
    }

    public Command driveFacingAngleCommand(DoubleSupplier vX, DoubleSupplier vY, Supplier<Rotation2d> angSupplier) {
        return applyRequest(() -> drive
                .withVelocityX(vX.getAsDouble())
                .withVelocityY(vY.getAsDouble())
                .withRotationalRate(HEADING_CONTROLLER.calculate(
                        getState().Pose.getRotation().getRadians(), angSupplier.get().getRadians())));
    }

    public void addVisionMeasurements(List<TimestampedVisionUpdate> updates) {
        for (int i = 0; i < updates.size(); i++) {
            addVisionMeasurement(updates.get(i).pose(), updates.get(i).timestamp(), updates.get(i).stdDevs());
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

    private boolean withinRange() {
        Pose2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.targetPoseBlue
                : FieldConstants.targetPoseRed;
        Pose2d robotPose = getState().Pose;

        Translation2d fieldRobotSpeeds = new Translation2d(
                getState().speeds.vxMetersPerSecond,
                getState().speeds.vyMetersPerSecond);

        /* Predict where target will be based on our current speeds */
        Translation2d virtualTarget = target.getTranslation()
                .plus(
                        fieldRobotSpeeds.times(ShooterConstants.tangentialNoteFlightTime)
                                .rotateBy(Rotation2d.fromDegrees(180.0)));

        return virtualTarget.getDistance(robotPose.getTranslation()) <= rangeTheshold;
    }

    public final Trigger withinRange = new Trigger(() -> withinRange());

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
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        pose2dPublisher.set(getState().Pose);
    }
}
