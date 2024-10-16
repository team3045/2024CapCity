// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotorLeft = new TalonFX(IntakeConstants.leftID, IntakeConstants.canbus);
  private TalonFX intakeMotorRight = new TalonFX(IntakeConstants.rightID, IntakeConstants.canbus);

  private enum IntakeState{
    INTAKING,
    IDLE
  }

  private double gearing = 1 / 1.5;

  private static final FlywheelSim LEFT_FLYWHEEL_SIM = new FlywheelSim(
    DCMotor.getKrakenX60(1),
    1.5,
    0.1
  );

   private static final FlywheelSim RIGHT_FLYWHEEL_SIM = new FlywheelSim(
    DCMotor.getKrakenX60(1),
    1.5,
    0.1
  );

  private TalonFXSimState leftSimState;
  private TalonFXSimState rightSimState;

  private IntakeState state;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotorLeft.getConfigurator().apply(
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    intakeMotorRight.getConfigurator().apply(
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive));
    
    state = IntakeState.IDLE;

    if(Utils.isSimulation()){
      configSim();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeMotor(){
    return this.run(() -> {
      System.out.println("\033[31mRunning Intake\033[39m");
      intakeMotorLeft.set(IntakeConstants.intakeSpeed);
      intakeMotorRight.set(IntakeConstants.intakeSpeed);
      state = IntakeState.INTAKING;
    });
  }

  public Command stopIntakeMotor(){
    return this.run(() -> {
      intakeMotorLeft.stopMotor();
      intakeMotorRight.stopMotor();
      state = IntakeState.IDLE;
    });
  }

  public Command setIntakingState(){
    return this.runOnce(() -> state = IntakeState.INTAKING);
  }

  public Command setIdleState(){
    return this.runOnce(() -> state = IntakeState.IDLE);
  }

  public void setIdleStateRunnable(){
    state = IntakeState.IDLE;
  }

  public void configSim() {
    leftSimState = intakeMotorLeft.getSimState();
    rightSimState = intakeMotorRight.getSimState();

    leftSimState.setSupplyVoltage(RobotController.getInputVoltage());
    rightSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    LEFT_FLYWHEEL_SIM.setState(0);
    RIGHT_FLYWHEEL_SIM.setState(0);
  }

  @Override
  public void simulationPeriodic(){
    leftSimState = intakeMotorLeft.getSimState();
    rightSimState = intakeMotorRight.getSimState();

    LEFT_FLYWHEEL_SIM.setInputVoltage(leftSimState.getMotorVoltage());
    RIGHT_FLYWHEEL_SIM.setInputVoltage(rightSimState.getMotorVoltage());
    LEFT_FLYWHEEL_SIM.update(0.02);
    RIGHT_FLYWHEEL_SIM.update(0.02);

    leftSimState.setRotorVelocity(LEFT_FLYWHEEL_SIM.getAngularVelocityRPM() / 60 * gearing);
    rightSimState.setRotorVelocity(RIGHT_FLYWHEEL_SIM.getAngularVelocityRPM() / 60 * gearing);

    SmartDashboard.putNumber("Intake Left Velocity", intakeMotorLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Right Velocity", intakeMotorRight.getVelocity().getValueAsDouble());
  }

  //TRIGGERS
  public final Trigger isIntaking = new Trigger(() -> state == IntakeState.INTAKING);
}
