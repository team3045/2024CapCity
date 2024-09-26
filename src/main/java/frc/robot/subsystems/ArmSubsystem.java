// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;


import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinUtil;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final CANcoder cancoder;
  private double setpoint = minAngle;

  //SIMULATION
  private TalonFXSimState leftMotorSimState;
  private TalonFXSimState rightMotorSimState;
  private CANcoderSimState cancoderSimState;

  private SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getKrakenX60(2), 
    totalGearing, 
    armMOI, 
    armCOM, 
    Units.degreesToRadians(minAngle),
    Units.degreesToRadians(maxAngle), 
    true, 
    Units.degreesToRadians(minAngle));

  //Dashboard 
  private MechanismLigament2d mechanismLigament2d;
  private MechanismRoot2d mechanismRoot;
  private Mechanism2d mechanism;

  //Publishing
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable armTable = inst.getTable("arm");
  private final StructPublisher<Rotation2d> rotation2dPublisher = armTable.getStructTopic("Arm Rotation2d", Rotation2d.struct).publish();

  //TRIGGERS
  // Add a trigger for isReady; debounce it so that it doesn't flicker while we're shooting
  // TODO: Consider caching.
  public final Trigger atTarget = new Trigger(this::atTargetPosition).debounce(atTargetDelay, DebounceType.kFalling);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    leftMotor = new TalonFX(leftMotorID, canbus);
    rightMotor = new TalonFX(rightMotorID, canbus);
    cancoder = new CANcoder(cancoderID, canbus);
    setpoint = getPositionDegrees();

    //Setup Mechanism
    mechanism = new Mechanism2d(canvasWidth, canvasHeight);
    mechanismRoot = mechanism.getRoot("pivot", rootX, rootY);
    mechanismLigament2d = mechanismRoot.append(
      new MechanismLigament2d("armLength", armCOM, getPositionDegrees())
    );
  }

  public void configMotors(){
    leftMotor.getConfigurator().apply(motorConfig.withMotorOutput(
      new MotorOutputConfigs().withInverted(leftInverted)));
    rightMotor.getConfigurator().apply(motorConfig.withMotorOutput(
      new MotorOutputConfigs().withInverted(rightInverted)));
    cancoder.getConfigurator().apply(cancoderConfig); 
  }

  //Ideally don't use, add periodic functions in RobotContainer
  @Override
  public void periodic() {}

  public double getPositionDegrees(){
    return Units.rotationsToDegrees(cancoder.getPosition().getValueAsDouble());
  }
  
  public double getVelocityDegPerSec(){
    return Units.rotationsToDegrees(cancoder.getVelocity().getValueAsDouble());
  }

  public void logPeriodic(){
    GremlinLogger.logTalonFX(path + "leftArmMotor", leftMotor);
    GremlinLogger.logTalonFX(path + "rightArmMotor", rightMotor);

    GremlinLogger.log(path + "Angle (Deg)", getPositionDegrees());
    GremlinLogger.log(path + "Velocity (Deg/Sec)", getVelocityDegPerSec());
    GremlinLogger.log(path + "Target Angle (Deg)", setpoint);
  }

  public boolean atTargetPosition(){
    return MathUtil.isNear(setpoint,getPositionDegrees(), angleTolerance) && 
          MathUtil.isNear(0, getVelocityDegPerSec(), velocityTolerance);
  }

  public void setTarget(double targetAngle){
    if(targetAngle > maxAngle){
      setpoint = maxAngle;
      GremlinLogger.logFault("Setpoint Exceeds Max Angle");
    } else if (targetAngle < minAngle){
      setpoint = minAngle;
      GremlinLogger.logFault("Setpoint Below Min Angle");
    } else{
      setpoint = targetAngle;
    }

    MotionMagicVoltage request = new MotionMagicVoltage(targetAngle)
      .withEnableFOC(true).withSlot(0).withUpdateFreqHz(500);

    leftMotor.setControl(request);
    rightMotor.setControl(request);
  }

  public Command goToAngle(DoubleSupplier angle){
    return this.run(() -> {
      setTarget(angle.getAsDouble());
    });
  }

  public Command goToIntake(){
    return goToAngle(() -> intakeAngle);
  }

  public Command goToAmp(){
    return goToAngle(() -> ampAngle);
  }

  public Rotation2d getAngleRotation2d(){
    return Rotation2d.fromDegrees(getPositionDegrees());
  }

  public void displayMechanism(){
    mechanismLigament2d.setAngle(getAngleRotation2d());
    SmartDashboard.putData(path + "Mechanism", mechanism);
  }

  public Command increaseAngle(){
    return goToAngle(() -> getPositionDegrees() + 5);
  }

  public Command decreaseAngle(){
    return goToAngle(() -> getPositionDegrees() - 5);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", () -> getVelocityDegPerSec(), null);
    builder.addDoubleProperty("Position", () -> getPositionDegrees(), null);
    builder.addBooleanProperty("At Target?", () -> atTargetPosition(), null);
    builder.addDoubleProperty("Setpoint", () -> setpoint, null);
  }

  @Override
  public void simulationPeriodic(){
    leftMotorSimState = leftMotor.getSimState();
    rightMotorSimState = rightMotor.getSimState();
    cancoderSimState = cancoder.getSimState();

    leftMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    armSim.setInputVoltage(leftMotorSimState.getMotorVoltage());
    armSim.update(0.02);

    leftMotorSimState.setRawRotorPosition(
      GremlinUtil.valueAfterGearing(Units.radiansToRotations(armSim.getAngleRads()), totalGearing));
    leftMotorSimState.setRotorVelocity(
      GremlinUtil.valueAfterGearing(Units.radiansToRotations(armSim.getVelocityRadPerSec()), totalGearing));

    rightMotorSimState.setRawRotorPosition(
      GremlinUtil.valueAfterGearing(Units.radiansToRotations(armSim.getAngleRads()), totalGearing));
    rightMotorSimState.setRotorVelocity(
      GremlinUtil.valueAfterGearing(Units.radiansToRotations(armSim.getVelocityRadPerSec()), totalGearing));

    cancoderSimState.setRawPosition(
      GremlinUtil.valueAfterGearing(Units.degreesToRotations(armSim.getAngleRads()), sensorToMechanismRatio));
    cancoderSimState.setVelocity(
      GremlinUtil.valueAfterGearing(Units.degreesToRotations(armSim.getVelocityRadPerSec()), sensorToMechanismRatio));
  

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps())
    );

    logPeriodic();
    displayMechanism();
  }
}
