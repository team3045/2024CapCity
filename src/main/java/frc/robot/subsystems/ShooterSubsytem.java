// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commons.DistanceSensorReader;
import frc.robot.commons.GremlinLogger;

public class ShooterSubsytem extends SubsystemBase {
  private final TalonFX leftShooter = new TalonFX(leftMotorID, canbus);
  private final TalonFX rightShooter = new TalonFX(rightMotorID, canbus);
  private final TalonFX feedMotor = new TalonFX(feedMotorID, canbus);
  private final DistanceSensorReader frontRangeSensor = new DistanceSensorReader(1);
  private final DistanceSensorReader backRangeSensor = new DistanceSensorReader(0);
  private final Notifier frontRangeSensorNotifer = new Notifier(frontRangeSensor);
  private final Notifier backRangeSensorNotifier = new Notifier(backRangeSensor);
  private boolean hasGamePieceFront;
  private boolean hasGamePieceBack;
  private double setpoint;
  private boolean defaultShot;

  public enum ShooterState{
    IDLE, 
    INTAKING,
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
    frontRangeSensor.run();
    frontRangeSensorNotifer.setName("Front Range Sensor");
    frontRangeSensorNotifer.startPeriodic(0.02); //runs on seperste thread to prevent loop overun
    backRangeSensor.run();
    backRangeSensorNotifier.setName("Back Range Sensor");
    backRangeSensorNotifier.startPeriodic(0.02);


    hasGamePieceFront = frontRangeSensor.getRange() < frontHasNoteThreshold; 
    hasGamePieceBack = backRangeSensor.getRange() < backHasNoteThreshold;

    configMotors();
    
    if(Utils.isSimulation()){
      configSim();
    }

    setpoint = 0;
    defaultShot = false;
  }

  public void configMotors(){
    leftShooter.getConfigurator().apply(config.withMotorOutput(
      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
    );
    rightShooter.getConfigurator().apply(config.withMotorOutput(
      new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
    ));
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

  public void setDefaultShot(boolean val){
    defaultShot = val;
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

  public Command runForwardSlow(){
    return this.run(() -> {
      setShooterSpeed(-10);
      feedMotor.set(0.1);
    });
  }

  public Command runBackSlow(){
    return this.run(() -> {
      setShooterSpeed(-10);
      feedMotor.set(-0.1);
    });
  }

  public Command ampShot(){
    return this.run(() ->{
      setShooterSpeed(-10);
      feedMotor.set(-0.3);
    });
  }

  

  public Command setRevving(){
    return this.run(() -> {
      mState = ShooterState.REVVING;
      setShooterSpeed(shootingVelocity);
    }).withName("Revving Motors");
  }

  public Command switchRevvingIdle(){
      if(mState == ShooterState.IDLE)
        return setRevving().alongWith(Commands.print("Idle to Revving"));
      else if (mState == ShooterState.REVVING)
        return Commands.print("Revving to Idle");
      else
        return Commands.print("Not in either Idle or Revving");
  }


  public Command feedNote(){
    return this.run(() -> {
      // MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(feedingVelocity)
      //   .withSlot(0).withUpdateFreqHz(50);
      // feedMotor.setControl(request);
      feedMotor.set(0.9);
    }).withName("Feeding Note");
  }

  public Command startIntaking(){
    return this.run(() -> {
      feedMotor.set(0.1);
      mState = ShooterState.INTAKING;
    });
  }
  
  public void stopIntaking(){
      feedMotor.set(0);
      mState = ShooterState.IDLE;
  }

  //shooters neutral mode should be coast
  public Command coastShootersAndIdle(){
    return this.runOnce(() -> {
      mState = ShooterState.IDLE;
      leftShooter.setControl(new NeutralOut());
      rightShooter.setControl(new NeutralOut());
      setDefaultShot(false);
    }).withName("Coast Shooters");
  }

  //shooters neutral mode should be coast
  public void coastShootersAndIdleRunnable(){
      mState = ShooterState.IDLE;
      leftShooter.setControl(new NeutralOut());
      rightShooter.setControl(new NeutralOut());
      setDefaultShot(false);
  }

  //feedmotor neutral mode should be brake
  public Command stopFeed(){
    return this.runOnce(() -> {
      feedMotor.stopMotor();
    }).withName("stopping feed");
  }

  public void stopAll(){
    feedMotor.stopMotor();
    leftShooter.stopMotor();
    rightShooter.stopMotor();
  }

  @Override
  public void periodic() {
    if(frontRangeSensor.isValid())
      hasGamePieceFront = frontRangeSensor.getRange() < frontHasNoteThreshold; //cache sensor value so its same every iteration
    if(backRangeSensor.isValid())
      hasGamePieceBack = backRangeSensor.getRange() < backHasNoteThreshold;
    logPeriodic();
  }

  public void logPeriodic(){
    GremlinLogger.logTalonFX(path + "leftShooter", leftShooter);
    GremlinLogger.logTalonFX(path + "rightShooter", rightShooter);
    GremlinLogger.logTalonFX(path + "feedMotor", feedMotor);

    GremlinLogger.logSD(path + "Has Note Front", hasNoteFront.getAsBoolean());
    GremlinLogger.logSD(path + "Has Note Back", hasNoteBack.getAsBoolean());
    GremlinLogger.logSD(path + "State", mState.toString());
    GremlinLogger.logSD(path + "At Speed", atSpeed.getAsBoolean());
    GremlinLogger.logSD(path + "Setpoint", setpoint);
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
  public final Trigger hasNoteFront = new Trigger(() -> hasGamePieceFront).debounce(hasNoteDebounceTime, DebounceType.kFalling);
  public final Trigger hasNoteBack = new Trigger(() -> hasGamePieceBack).debounce(hasNoteDebounceTime, DebounceType.kFalling);
  public final Trigger atSpeed = new Trigger(() -> atTargetSpeed()).debounce(hasNoteDebounceTime, DebounceType.kFalling);

  //isShooting is only enabled after revving has finished so we don't need to check for speed
  public final Trigger shouldFeed = isShooting.and(hasNoteFront);

  public final Trigger defaultShotTrigger = new Trigger(() -> defaultShot);

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

  public void applyVoltageShooter(double volts){
    leftShooter.setVoltage(volts);
    rightShooter.setVoltage(volts);
  }

  //SYSID
  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Angle> appliedAngle = MutableMeasure.mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> appliedVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));
  private final MutableMeasure<Current> appliedCurrent = MutableMeasure.mutable(Amps.of(0));
  private final Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
  private SysIdRoutine shooterRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        rampRate, 
        Volts.of(7), 
        Seconds.of(10)), 
      new SysIdRoutine.Mechanism(
        (volts) -> applyVoltageShooter(volts.in(Volts)),
        (log) -> {
          log.motor("LeftSideMotor")
          .voltage(appliedVoltage.mut_replace(leftShooter.getMotorVoltage().getValueAsDouble(), Volts))
          .angularVelocity(appliedVelocity.mut_replace(getCurrentSpeedLeft(), RotationsPerSecond))
          .angularPosition(appliedAngle.mut_replace(leftShooter.getPosition().getValueAsDouble(), Rotations))
          .current(appliedCurrent.mut_replace(leftShooter.getTorqueCurrent().getValueAsDouble(), Amps));
        log.motor("RightSideMotor")
          .voltage(appliedVoltage.mut_replace(rightShooter.getMotorVoltage().getValueAsDouble(), Volts))
          .angularVelocity(appliedVelocity.mut_replace(getCurrentSpeedRight(), RotationsPerSecond))
          .angularPosition(appliedAngle.mut_replace(rightShooter.getPosition().getValueAsDouble(), Rotations))
          .current(appliedCurrent.mut_replace(rightShooter.getTorqueCurrent().getValueAsDouble(), Amps));
        }, 
         this)
    );

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return shooterRoutine.dynamic(direction);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return shooterRoutine.quasistatic(direction);
  }


}
