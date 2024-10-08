// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static frc.robot.constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commons.DistanceSensorReader;
import frc.robot.commons.GremlinLogger;

public class ShooterSubsytem extends SubsystemBase {
  private final TalonFX leftShooter;
  private final TalonFX rightShooter;
  private final TalonFX feedMotor;
  private final DistanceSensorReader rangeSensor = new DistanceSensorReader();
  private final Notifier rangeSensorNotifer = new Notifier(rangeSensor);
  private boolean hasGamePiece;
  private double setpoint;

  public enum ShooterState{
    IDLE, 
    REVVING,
    SHOOTING,
    AMP,
    EJECTING
  }

  private ShooterState mState = ShooterState.IDLE;

  private static final FlywheelSim LEFT_FLYWHEEL_SIM = new FlywheelSim(
    DCMotor.getKrakenX60(1), 
    gearing, 
    momentOfInertia);

  private static final FlywheelSim RIGHT_FLYWHEEL_SIM = new FlywheelSim(
    DCMotor.getKrakenX60(1), 
    gearing, 
    momentOfInertia);

  private TalonFXSimState leftSimState;
  private TalonFXSimState rightSimState;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {
    rangeSensor.run();
    rangeSensorNotifer.setName("Range Sensor");
    rangeSensorNotifer.startPeriodic(0.02); //runs on seperste thread to prevent loop overun

    leftShooter = new TalonFX(leftMotorID, canbus);
    rightShooter = new TalonFX(rightMotorID, canbus);
    feedMotor = new TalonFX(feedMotorID, canbus);

    hasGamePiece = rangeSensor.getRange() < hasNoteThreshold; 

    configMotors();
    
    if(Utils.isSimulation()){
      configSim();
    }

    setpoint = 0;
  }

  public void configMotors(){
    leftShooter.getConfigurator().apply(config);
    rightShooter.getConfigurator().apply(config);
  }

  public double getCurrentSpeedLeft(){
     return leftShooter.getVelocity().getValueAsDouble();
  }

  public double getCurrentSpeedRight(){
    return rightShooter.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed(){
    return MathUtil.isNear(setpoint, getCurrentSpeedLeft(), speedErrorTolerance)
      && MathUtil.isNear(setpoint, getCurrentSpeedRight(), speedErrorTolerance);
  }

  //This is the internal command factory to set state
  private Command setState(ShooterState newState){
    return Commands.runOnce(() -> {
      mState = newState;
    }, this).withName(newState.toString());
  }

  //public commands to set state
  public Command setIdle(){
    return setState(ShooterState.IDLE);
  }

  public Command setShooting(){
    return setState(ShooterState.SHOOTING);
  }

  public Command setAmp(){
    return setState(ShooterState.AMP);
  }

  public Command setEjecting(){
    return setState(ShooterState.EJECTING);
  }

  public void setShooterSpeed(double requestedVeloRPS){
    setpoint = requestedVeloRPS;

    MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(requestedVeloRPS)
      .withEnableFOC(false).withUpdateFreqHz(500).withSlot(0);
    leftShooter.setControl(request);
    rightShooter.setControl(request);
  }

  public Command setRevving(){
    return this.runOnce(() -> {
      mState = ShooterState.REVVING;
      setShooterSpeed(shootingVelocity);
    }).withName("Revving Motors");
  }

  public Command feedNote(){
    return this.run(() -> {
      MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(feedingVelocity)
        .withSlot(0).withUpdateFreqHz(50);
      feedMotor.setControl(request);
    }).withName("Feeding Note");
  }

  //shooters neutral mode should be coast
  public Command coastShootersAndIdle(){
    return this.run(() -> {
      mState = ShooterState.IDLE;
      leftShooter.setControl(new NeutralOut());
      rightShooter.setControl(new NeutralOut());
    }).withName("Coast Shooters");
  }

  //feedmotor neutral mode should be brake
  public Command stopFeed(){
    return this.run(() -> {
      feedMotor.stopMotor();
    }).withName("stopping feed");
  }

  @Override
  public void periodic() {
    hasGamePiece = rangeSensor.getRange() < hasNoteThreshold; //cache sensor value so its same every iteration
    logPeriodic();
  }

  public void logPeriodic(){
    GremlinLogger.logTalonFX(path + "leftShooter", leftShooter);
    GremlinLogger.logTalonFX(path + "rightShooter", rightShooter);
    GremlinLogger.logTalonFX(path + "feedMotor", feedMotor);

    GremlinLogger.log(path + "LeftShooter/Velocity", getCurrentSpeedLeft());
    GremlinLogger.log(path + "RightShooter/Velocity", getCurrentSpeedRight());
    GremlinLogger.log(path + "Has Note", hasNote.getAsBoolean());
    GremlinLogger.log(path + "State", mState.toString());

    SmartDashboard.putNumber(path + "LeftShooter/Velocity", getCurrentSpeedLeft());
    SmartDashboard.putNumber(path + "RightShooter/Velocity", getCurrentSpeedRight());
    SmartDashboard.putBoolean(path + "Has Note", hasNote.getAsBoolean());
    SmartDashboard.putBoolean(path + "At Speed", atSpeed.getAsBoolean());
    SmartDashboard.putString(path + "State", mState.toString());
  }

  //TRIGGERS
  public final Trigger isIdle = new Trigger(() -> mState == ShooterState.IDLE);
  public final Trigger isRevving = new Trigger(() -> mState == ShooterState.REVVING);
  public final Trigger isShooting = new Trigger(() -> mState == ShooterState.SHOOTING);
  public final Trigger isAmp = new Trigger(() -> mState == ShooterState.AMP);
  public final Trigger isEjecting = new Trigger(() -> mState == ShooterState.EJECTING);

  // This trigger will be true when there is a game piece in the shooter
  // and stay true for a short time after no game piece is detected.
  // This delay allows the game piece to fully leave the shooter before we power down and stop aiming.
  //The second trigger will stay try for a short time after note leaves because the velocity drops slightly as the note leaves
  public final Trigger hasNote = new Trigger(() -> hasGamePiece).debounce(hasNoteDebounceTime, DebounceType.kFalling);
  public final Trigger atSpeed = new Trigger(() -> atTargetSpeed()).debounce(hasNoteDebounceTime, DebounceType.kFalling);

  //isShooting is only enabled after revving has finished so we don't need to check for speed
  public final Trigger shouldFeed = isShooting.and(hasNote);

  public void configSim(){
    leftSimState = leftShooter.getSimState();
    rightSimState = rightShooter.getSimState();

    leftSimState.setSupplyVoltage(RobotController.getInputVoltage());
    rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    LEFT_FLYWHEEL_SIM.setState(0);
    RIGHT_FLYWHEEL_SIM.setState(0);
  }

  @Override
  public void simulationPeriodic(){
    leftSimState = leftShooter.getSimState();
    rightSimState = rightShooter.getSimState();

    LEFT_FLYWHEEL_SIM.setInputVoltage(leftSimState.getMotorVoltage());
    RIGHT_FLYWHEEL_SIM.setInputVoltage(rightSimState.getMotorVoltage());
    LEFT_FLYWHEEL_SIM.update(0.02);
    RIGHT_FLYWHEEL_SIM.update(0.02);

    leftSimState.setRotorVelocity(LEFT_FLYWHEEL_SIM.getAngularVelocityRPM() / 60 * gearing);
    rightSimState.setRotorVelocity(RIGHT_FLYWHEEL_SIM.getAngularVelocityRPM() / 60 * gearing);

    logPeriodic();
  }
}