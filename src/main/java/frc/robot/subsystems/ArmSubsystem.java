// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.constants.ArmConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
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
      new MechanismLigament2d("armLength", armCOM, -minAngle + mech2dOffset) //IRL our 0 degrees is hanging straight down, 
                                                                          //but in the mechanism 2d its at a right angle
    );

    if(Utils.isSimulation()){
      configSim();
    }
  }

  /**
   * Config all devices so we don't rely on Tuner X configs. Called on subsystem construction. 
   */
  public void configDevices(){
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
  }

  //Ideally don't use, add periodic functions in RobotContainer
  @Override
  public void periodic() {
    displayMechanism();
    logPeriodic();
  }

  //TODO: Change to differential
  /**
   * @return returns current position of arm in degrees
   */
  public double getPositionDegrees(){
    double position = leftMotor.getPosition().getValueAsDouble();
    return Units.rotationsToDegrees(position);
  }
  
  /**
   * @return the current velocity of the arm in Degs / Sec 
   */
  public double getVelocityDegPerSec(){
    double velocity = leftMotor.getVelocity().getValueAsDouble();
    return Units.rotationsToDegrees(velocity);
  }

  /**
   * Periodically log as well as put important metrics on SmartDashboard. 
   * Values ionclude Current Arm Angle, Velocity, and Setpoint
   * as well as indiviudal Motor Metrics
   */
  public void logPeriodic(){
    GremlinLogger.logTalonFX(path + "leftArmMotor", leftMotor);
    GremlinLogger.logTalonFX(path + "rightArmMotor", rightMotor);

    GremlinLogger.log(path + "Angle (Deg)", getPositionDegrees());
    GremlinLogger.log(path + "Velocity (Deg per Sec)", getVelocityDegPerSec());
    GremlinLogger.log(path + "Target Angle (Deg)", setpoint);

    SmartDashboard.putNumber(path + "Angle (Deg)", getPositionDegrees());
    SmartDashboard.putNumber(path + "Velocity (Deg per S)", getVelocityDegPerSec());
    SmartDashboard.putNumber(path + "Target Angle (Deg)", setpoint);
  }

  /**
   * Checks if the arm is near the setpoint.
   *  Arm is considered near the setpoint if its within a constant tolerance of 
   * {@value frc.robot.constants.ArmConstants#angleTolerance} degrees
   * @return True if arm is within tolerance. False if arm is not within tolerance
   */
  public boolean atTargetPosition(){
    return MathUtil.isNear(setpoint,getPositionDegrees(), angleTolerance) && 
          MathUtil.isNear(0, getVelocityDegPerSec(), velocityTolerance);
  }


  /**
   * Internal Method to set the target position of the arm. 
   * Should only be acessed externally through command factories
   * 
   * @param targetAngle desired angle of the arm in degrees
   */
  private void setTarget(double targetAngle){
    setpoint = GremlinUtil.clampWithLogs(maxAngle, minAngle, targetAngle);

    MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(setpoint))
      .withEnableFOC(false).withSlot(0).withUpdateFreqHz(50);

    leftMotor.setControl(request);
    rightMotor.setControl(request);
  }

  /**
   * Basic command factory to send the arm to a specified angle
   * @param angle desired angle of the arm in degrees
   * @return A command controlling the arm to travel to the specified angle
   */
  public Command goToAngle(DoubleSupplier angle){
    return this.runOnce(() -> {
      setTarget(angle.getAsDouble());
    });
  }

  /**
   * @return returns a command controlling the arm to travel to the Intake position
   */
  public Command goToIntake(){
    return goToAngle(() -> intakeAngle);
  }

  /**
   * @return returns a command controlling the arm to travel to the amp position
   */
  public Command goToAmp(){
    return goToAngle(() -> ampAngle);
  }

  /**
   * Gets the arm angle in the form of a Rotation2d
   * @return a Rotation2d representing the current angle of the arm
   */
  public Rotation2d getAngleRotation2d(){
    return Rotation2d.fromDegrees(getPositionDegrees());
  }

  /**
   * Displays the arm as a 2d Widget on Smartdashboard
   */
  public void displayMechanism(){
    mechanismLigament2d.setAngle(getAngleRotation2d().times(-1).minus(Rotation2d.fromDegrees(mech2dOffset)));
    SmartDashboard.putData(path + "Mechanism", mechanism);
  }

  /**
   * Increases the arm angle by 5 degrees
   * @return A command to increase the arm angle by 5 degree
   */
  public Command increaseAngle(){
    return goToAngle(() -> getPositionDegrees() + 5); 
  }

  /**
   * Decreases the arm angle by 5 degrees
   * @return A command to decrease the arm angle by 5 degree
   */
  public Command decreaseAngle(){
    return goToAngle(() -> getPositionDegrees() - 5);
  }

  /**
   * Configures Simulation
   */
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

    cancoderSimState.setRawPosition(Units.radiansToRotations(angle * sensorToMechanismRatio));
    rightMotorSimState.setRawRotorPosition(Units.radiansToRotations(angle * totalGearing));
    leftMotorSimState.setRawRotorPosition(Units.radiansToRotations(angle * totalGearing));

    rightMotorSimState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() * totalGearing));
    leftMotorSimState.setRotorVelocity(Units.radiansToRotations(armSim.getVelocityRadPerSec() * totalGearing));

    logPeriodic();
    displayMechanism();
  }
}
