// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.ArmAngles;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX leftMotor = new TalonFX(leftMotorID, canbus);
  private TalonFX rightMotor = new TalonFX(rightMotorID, canbus);
  private CANcoder cancoder = new CANcoder(cancoderID, canbus);
  private double setpoint = minAngle;
  private boolean zeroed = false;

  // SIMULATION
  private TalonFXSimState leftMotorSimState;
  private TalonFXSimState rightMotorSimState;
  private CANcoderSimState cancoderSimState;

  private static SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(2),
      totalGearing,
      armMOI,
      armCOM,
      Units.degreesToRadians(minAngle),
      Units.degreesToRadians(maxAngle),
      true,
      Units.degreesToRadians(minAngle));

  // Dashboard
  private MechanismLigament2d mechanismLigament2d;
  private MechanismRoot2d mechanismRoot;
  private Mechanism2d mechanism;

  // Publishing
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable armTable = inst.getTable("Positioner");
  private final StructPublisher<Pose3d> pose3dPublisher = armTable.getStructTopic("Arm Pose3d", Pose3d.struct)
      .publish();

  // TRIGGERS
  // Add a trigger for isReady; debounce it so that it doesn't flicker while we're
  // shooting
  // TODO: Consider caching.
  public final Trigger atTarget = new Trigger(this::atTargetPosition).debounce(atTargetDelay, DebounceType.kFalling);
  public final Trigger atIntake = new Trigger(this::atIntake).debounce(atTargetDelay, DebounceType.kFalling);

  // SYSID
  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Angle> appliedAngle = MutableMeasure.mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> appliedVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));
  private final MutableMeasure<Current> appliedCurrent = MutableMeasure.mutable(Amps.of(0));
  private final Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
  private SysIdRoutine armRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
          rampRate,
          Volts.of(4),
          Seconds.of(10),
          null),
      new SysIdRoutine.Mechanism(
          (volts) -> applyVoltage(volts.in(Volts)),
          log -> {
            log.motor("LeftSideMotor")
                .voltage(appliedVoltage.mut_replace(leftMotor.getMotorVoltage().getValueAsDouble(), Volts))
                .angularPosition(appliedAngle.mut_replace(getPositionRotations(), Rotations))
                .angularVelocity(appliedVelocity.mut_replace(getVelocityRotPerSec(), RotationsPerSecond))
                .current(appliedCurrent.mut_replace(leftMotor.getTorqueCurrent().getValueAsDouble(), Amps));
            log.motor("RightSideMotor")
                .voltage(appliedVoltage.mut_replace(leftMotor.getMotorVoltage().getValueAsDouble(), Volts))
                .angularPosition(appliedAngle.mut_replace(getPositionRotations(), Rotations))
                .angularVelocity(appliedVelocity.mut_replace(getVelocityRotPerSec(), RotationsPerSecond))
                .current(appliedCurrent.mut_replace(leftMotor.getTorqueCurrent().getValueAsDouble(), Amps));
          },
          this));

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    configDevices();
    setpoint = minAngle;

    // Setup Mechanism
    mechanism = new Mechanism2d(canvasWidth, canvasHeight);
    mechanismRoot = mechanism.getRoot("pivot", rootX, rootY);
    mechanismLigament2d = mechanismRoot.append(
        new MechanismLigament2d("armLength", armCOM, -minAngle + mech2dOffset) // offset so its to the left, just
                                                                               // personal preference
    );

    if (Utils.isSimulation()) {
      configSim();
    }
  }

  /**
   * Config all devices so we don't rely on Tuner X configs. Called on subsystem
   * construction.
   */
  public void configDevices() {
    leftMotor.getConfigurator().apply(motorConfig.withMotorOutput(
        new MotorOutputConfigs().withInverted(leftInverted).withNeutralMode(NeutralModeValue.Brake)));
    rightMotor.getConfigurator().apply(motorConfig.withMotorOutput(
        new MotorOutputConfigs().withInverted(rightInverted).withNeutralMode(NeutralModeValue.Brake)));
    cancoder.getConfigurator().apply(cancoderConfig);

    leftMotor.clearStickyFaults();
    rightMotor.clearStickyFaults();
    cancoder.clearStickyFaults();
    leftMotor.clearStickyFault_RemoteSensorDataInvalid();
    rightMotor.clearStickyFault_RemoteSensorDataInvalid();

    cancoder.setPosition(Units.degreesToRotations(minAngle) * sensorToMechanismRatio);
  }

  // Ideally don't use, add periodic functions in RobotContainer
  @Override
  public void periodic() {
    displayMechanism();
    logPeriodic();
  }

  /**
   * @return returns current position of arm in degrees
   */
  public double getPositionDegrees() {
    double position = cancoder.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(position / sensorToMechanismRatio);
  }

  public double getPositionRotations() {
    return getPositionDegrees() / 360;
  }

  /**
   * @return returns current position of arm in Radians
   */
  public double getPositionRadians() {
    double position = leftMotor.getPosition().getValueAsDouble();
    return Units.rotationsToRadians(position);
  }

  /**
   * @return the current velocity of the arm in Degs / Sec
   */
  public double getVelocityDegPerSec() {
    double velocity = cancoder.getVelocity().getValueAsDouble();
    return Units.rotationsToDegrees(velocity / sensorToMechanismRatio);
  }

  public double getVelocityRotPerSec() {
    return getVelocityDegPerSec() / 360;
  }

  /**
   * Periodically log as well as put important metrics on SmartDashboard.
   * Values ionclude Current Arm Angle, Velocity, and Setpoint
   * as well as indiviudal Motor Metrics
   */
  public void logPeriodic() {
    GremlinLogger.logTalonFX(path + "leftArmMotor", leftMotor);
    GremlinLogger.logTalonFX(path + "rightArmMotor", rightMotor);

    GremlinLogger.log(path + "Angle (Deg)", getPositionDegrees());
    GremlinLogger.log(path + "Velocity (Deg per Sec)", getVelocityDegPerSec());
    GremlinLogger.log(path + "Target Angle (Deg)", setpoint);
    GremlinLogger.log(path + "At Position", atTarget.getAsBoolean());

    SmartDashboard.putNumber(path + "Angle (Deg)", getPositionDegrees());
    SmartDashboard.putNumber(path + "Velocity (Deg per S)", getVelocityDegPerSec());
    SmartDashboard.putNumber(path + "Target Angle (Deg)", setpoint);
    SmartDashboard.putBoolean(path + "At Position", atTarget.getAsBoolean());
    SmartDashboard.putBoolean(path + "At Intake", atIntake.getAsBoolean());
  }

  /**
   * Checks if the arm is near the setpoint.
   * Arm is considered near the setpoint if its within a constant tolerance of
   * {@value frc.robot.constants.ArmConstants#angleTolerance} degrees
   * 
   * @return True if arm is within tolerance. False if arm is not within tolerance
   */
  public boolean atTargetPosition() {
    return MathUtil.isNear(setpoint, getPositionDegrees(), angleTolerance) &&
        MathUtil.isNear(0, getVelocityDegPerSec(), velocityTolerance);
  }

  public boolean atIntake() {
    return MathUtil.isNear(intakeAngle, getPositionDegrees(), angleTolerance) &&
        MathUtil.isNear(0, getVelocityDegPerSec(), velocityTolerance);
  }

  public void findZero() {
    if (zeroed) {
      return;
    }
    leftMotor.setVoltage(-1.0);
    rightMotor.setVoltage(-1.0);

    double time = Timer.getFPGATimestamp();
    Timer.delay(0.1);

    while (true) {
      if (MathUtil.isNear(0, leftMotor.getVelocity().getValueAsDouble(), 0.1)) {
        break;
      }
      if (Timer.getFPGATimestamp() - time >= 5) { // 5seconds timeout
        break;
      }
    }
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
    Timer.delay(0.3);
    cancoder.setPosition(Units.degreesToRotations(minAngle) * sensorToMechanismRatio);
    setpoint = getPositionDegrees();
  }

  /**
   * Internal Method to set the target position of the arm.
   * Should only be acessed externally through command factories
   * 
   * @param targetAngle desired angle of the arm in degrees
   */
  private void setTarget(double targetAngle) {
    setpoint = GremlinUtil.clampWithLogs(maxAngle, minAngle, targetAngle);

    double gravFeedforward = kG * Math.cos(getPositionRadians());
    System.out.println("Grav feedforward: " + gravFeedforward);
    System.out.println("Current cos: " + Math.cos(getPositionRadians()));

    MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(setpoint))
        .withEnableFOC(false).withSlot(0).withUpdateFreqHz(50)
        .withFeedForward(gravFeedforward);

    leftMotor.setControl(request);
    rightMotor.setControl(request);
  }

  /**
   * Basic command factory to send the arm to a specified angle
   * 
   * @param angle desired angle of the arm in degrees
   * @return A command controlling the arm to travel to the specified angle
   */
  public Command goToAngle(DoubleSupplier angle) {
    return this.runOnce(() -> {
      setTarget(angle.getAsDouble());
    }).until(() -> atTargetPosition());
  }

  /**
   * @return returns a command controlling the arm to travel to the Intake
   *         position
   */
  public Command goToIntake() {
    return goToAngle(() -> intakeAngle);
  }

  /**
   * @return returns a command controlling the arm to travel to the amp position
   */
  public Command goToAmp() {
    return goToAngle(() -> ampAngle);
  }

  public Command goToMin() {
    return goToAngle(() -> minAngle);
  }

  public Command goToMax() {
    return goToAngle(() -> maxAngle);
  }

  /**
   * Gets the arm angle in the form of a Rotation2d
   * 
   * @return a Rotation2d representing the current angle of the arm
   */
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromDegrees(getPositionDegrees());
  }

  /**
   * Displays the arm as a 2d Widget on Smartdashboard
   */
  public void displayMechanism() {
    mechanismLigament2d.setAngle(getAngleRotation2d().times(-1).minus(Rotation2d.fromDegrees(mech2dOffset)));
    SmartDashboard.putData(path + "Mechanism", mechanism);

    double pitch = getPositionRadians();

    pose3dPublisher.set(new Pose3d(
        new Translation3d(simPositionX, simPositionY, simPositionZ),
        new Rotation3d(0, -pitch, 0)));
  }

  /**
   * Increases the arm angle by 5 degrees
   * 
   * @return A command to increase the arm angle by 5 degree
   */
  public Command increaseAngle() {
    return goToAngle(() -> getPositionDegrees() + 5);
  }

  /**
   * Decreases the arm angle by 5 degrees
   * 
   * @return A command to decrease the arm angle by 5 degree
   */
  public Command decreaseAngle() {
    return goToAngle(() -> getPositionDegrees() - 5);
  }

  public Command setAngleFromDistance(DoubleSupplier distance) {
    return goToAngle(() -> getAngleFromDistance(distance));
  }

  public double getAngleFromDistance(DoubleSupplier distance) {
    return ArmAngles.map.get(distance.getAsDouble());
  }

  public void setCoast() {
    leftMotor.setControl(new CoastOut());
    rightMotor.setControl(new CoastOut());
  }

  /**
   * Configures Simulation
   */
  public void configSim() {
    armSim.setState(0, 0);
    leftMotorSimState = leftMotor.getSimState();
    rightMotorSimState = rightMotor.getSimState();
    cancoderSimState = cancoder.getSimState();

    leftMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    leftMotorSimState.Orientation = ChassisReference.Clockwise_Positive;
    rightMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    cancoderSimState.Orientation = ChassisReference.Clockwise_Positive;
  }

  @Override
  public void simulationPeriodic() {
    leftMotorSimState = leftMotor.getSimState();
    rightMotorSimState = rightMotor.getSimState();
    cancoderSimState = cancoder.getSimState();

    armSim.setInputVoltage(leftMotorSimState.getMotorVoltage());
    armSim.update(0.020);

    double angle = armSim.getAngleRads();

    cancoderSimState.setRawPosition(Units.radiansToRotations(angle * sensorToMechanismRatio));
    rightMotorSimState.setRawRotorPosition(Units.radiansToRotations(angle * totalGearing));
    leftMotorSimState.setRawRotorPosition(Units.radiansToRotations(angle * totalGearing));

    rightMotorSimState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() * totalGearing));
    leftMotorSimState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() * totalGearing));

    logPeriodic();
    displayMechanism();
  }

  // for sysid and characterization / testing
  public void applyVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  // SYSID COMMANDS
  public Command sysIdDynamicForward() {
    return armRoutine.dynamic(Direction.kForward)
        .until(() -> getPositionDegrees() >= maxAngle);
  }

  public Command sysIdDynamicReverse() {
    return armRoutine.dynamic(Direction.kReverse)
        .until(() -> getPositionDegrees() <= minAngle + 20);
  }

  public Command sysIdQuasistaticForward() {
    return armRoutine.quasistatic(Direction.kForward)
        .until(() -> getPositionDegrees() >= maxAngle);
  }

  public Command sysIdQuasistaticReverse() {
    return armRoutine.quasistatic(Direction.kReverse)
        .until(() -> getPositionDegrees() <= minAngle + 20);
  }
}
