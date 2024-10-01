// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.GremlinLogger;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX leftMotor = new TalonFX(leftMotorID, canbus);
  private TalonFX rightMotor = new TalonFX(rightMotorID, canbus);
  private CANcoder cancoder = new CANcoder(cancoderID, canbus);
  private double setpoint = minAngle;

  //SIMULATION
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
    configDevices();
    setpoint = 0;

    //Setup Mechanism
    mechanism = new Mechanism2d(canvasWidth, canvasHeight);
    mechanismRoot = mechanism.getRoot("pivot", rootX, rootY);
    mechanismLigament2d = mechanismRoot.append(
      new MechanismLigament2d("armLength", armCOM, -minAngle + mech2dOffset)
    );

    if(Utils.isSimulation()){
      configSim();
    }
  }

  public void configDevices(){

    leftMotor.getConfigurator().apply(motorConfig.withMotorOutput(
      new MotorOutputConfigs().withInverted(leftInverted)));
    rightMotor.getConfigurator().apply(motorConfig.withMotorOutput(
      new MotorOutputConfigs().withInverted(rightInverted)));
    cancoder.getConfigurator().apply(cancoderConfig); 

    leftMotor.clearStickyFaults();
    rightMotor.clearStickyFaults();
    cancoder.clearStickyFaults();
    leftMotor.clearStickyFault_RemoteSensorDataInvalid();
    rightMotor.clearStickyFault_RemoteSensorDataInvalid();
  }

  //Ideally don't use, add periodic functions in RobotContainer
  @Override
  public void periodic() {}

  //TODO: Change to differential
  public double getPositionDegrees(){
    double position = leftMotor.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(position);
  }
  
  public double getVelocityDegPerSec(){
    double velocity = leftMotor.getVelocity().getValueAsDouble();
    return Units.rotationsToDegrees(velocity);
  }

  public void logPeriodic(){
    // GremlinLogger.logTalonFX(path + "leftArmMotor", leftMotor);
    // GremlinLogger.logTalonFX(path + "rightArmMotor", rightMotor);

    // GremlinLogger.log(path + "Angle (Deg)", getPositionDegrees());
    // GremlinLogger.log(path + "Velocity (Deg per Sec)", getVelocityDegPerSec());
    // GremlinLogger.log(path + "Target Angle (Deg)", setpoint);

    SmartDashboard.putNumber(path + "Angle (Deg)", getPositionDegrees());
    SmartDashboard.putNumber(path + "Velocity (Deg per S)", getVelocityDegPerSec());
    SmartDashboard.putNumber(path + "Target Angle (Deg)", setpoint);
  }

  public boolean atTargetPosition(){
    return MathUtil.isNear(setpoint,getPositionDegrees(), angleTolerance) && 
          MathUtil.isNear(0, getVelocityDegPerSec(), velocityTolerance);
  }

  public void setTarget(double targetAngle){
    if(targetAngle > maxAngle){
      setpoint = maxAngle;
      GremlinLogger.logFault("Setpoint Exceeds Max Angle");
      System.out.println("too High");
    } else if (targetAngle < minAngle){
      setpoint = minAngle;
      GremlinLogger.logFault("Setpoint Below Min Angle");
      System.out.println("too low");
    } else{
      setpoint = targetAngle;
    }

    System.out.println("Setpoint: " + setpoint);

    MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(setpoint))
      .withEnableFOC(false).withSlot(0).withUpdateFreqHz(50);

    leftMotor.setControl(request);
    rightMotor.setControl(request);
  }

  public Command goToAngle(DoubleSupplier angle){
    return this.runOnce(() -> {
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
    mechanismLigament2d.setAngle(getAngleRotation2d().times(-1).minus(Rotation2d.fromDegrees(mech2dOffset)));
    SmartDashboard.putData(path + "Mechanism", mechanism);
  }

  public Command increaseAngle(){
    return goToAngle(() -> getPositionDegrees() + 5).alongWith(Commands.print("Position: " + getPositionDegrees())); //position is 0 IDK what bug is
  }

  public Command decreaseAngle(){
    return goToAngle(() -> getPositionDegrees() - 5).alongWith(Commands.print("Position: " + getPositionDegrees()));
  }

  public void configSim(){
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
  public void simulationPeriodic(){
    leftMotorSimState = leftMotor.getSimState();
    rightMotorSimState = rightMotor.getSimState();
    cancoderSimState = cancoder.getSimState();

    armSim.setInputVoltage(leftMotorSimState.getMotorVoltage());
    armSim.update(0.020);

    double angle = armSim.getAngleRads();

    cancoderSimState.setRawPosition(Units.radiansToRotations(angle / sensorToMechanismRatio));
    rightMotorSimState.setRawRotorPosition(Units.radiansToRotations(angle / totalGearing));
    leftMotorSimState.setRawRotorPosition(Units.radiansToRotations(angle / totalGearing));

    rightMotorSimState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() / totalGearing));
    leftMotorSimState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() / totalGearing));

    System.out.println("rotor set to: " + Units.radiansToRotations(angle / totalGearing));
    System.out.println("Arm Angle: " + Units.radiansToDegrees(angle));
    System.out.println("Angle cancoder: " + Units.rotationsToDegrees(cancoder.getPosition().getValueAsDouble() * sensorToMechanismRatio));
    System.out.println("Angle Motor: " + Units.rotationsToDegrees(leftMotor.getPosition().getValueAsDouble()));
    System.out.println("Motor Voltage: " + leftMotor.getMotorVoltage());
    System.out.println();

    logPeriodic();
    displayMechanism();
  }
}
