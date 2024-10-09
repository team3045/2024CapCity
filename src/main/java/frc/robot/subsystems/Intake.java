// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorId, IntakeConstants.canbus);
  private TalonFX feedMotor = new TalonFX(IntakeConstants.feedMotorId, ShooterConstants.canbus);


  /** Creates a new Intake. */
  public Intake() {
    feedMotor.getConfigurator().apply(
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    intakeMotor.getConfigurator().apply(
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeMotor(){
    return this.run(() -> intakeMotor.set(IntakeConstants.intakeSpeed));
  }

  public Command runFeedMotor(){
    return this.run(() -> feedMotor.set(IntakeConstants.feedSpeed));
  }

  public Command runBoth(){
    return this.run(() -> {
      feedMotor.set(IntakeConstants.feedSpeed);
      intakeMotor.set(IntakeConstants.intakeSpeed);
    });
  }

  public Command stopIntakeMotor(){
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command stopFeedMotor(){
    return this.run(() -> feedMotor.stopMotor());
  }

  public Command stopBoth(){
    return this.run(() -> {
      feedMotor.stopMotor();
      intakeMotor.stopMotor();
    });
  }

  public void stop(){
    feedMotor.stopMotor();
    intakeMotor.stopMotor();
  }

  public void stopFeedRunnable(){
    feedMotor.stopMotor();
  }

  public Command runBack(){
    return this.run(() -> feedMotor.set(-0.1));
  }
}
