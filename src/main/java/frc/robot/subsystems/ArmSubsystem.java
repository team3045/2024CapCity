// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final CANcoder cancoder;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    leftMotor = new TalonFX(ArmConstants.leftMotorID, ArmConstants.canbus);
    rightMotor = new TalonFX(ArmConstants.rightMotorID, ArmConstants.canbus);
    cancoder = new CANcoder(ArmConstants.cancoderID, ArmConstants.canbus);
  }

  public void configMotors(){
    leftMotor.getConfigurator().apply(ArmConstants.config.withMotorOutput(
      new MotorOutputConfigs().withInverted(ArmConstants.leftInverted)));
    rightMotor.getConfigurator().apply(ArmConstants.config.withMotorOutput(
      new MotorOutputConfigs().withInverted(ArmConstants.rightInverted)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
