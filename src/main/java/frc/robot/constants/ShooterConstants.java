// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.commons.GremlinUtil;

/** Add your docs here. */
public class ShooterConstants {
    public static final int leftMotorID = 14;
    public static final int rightMotorID = 15;
    public static final int feedMotorID = 16;
    public static final String canbus = "rio";

    public static final double hasNoteThreshold = 10; //cm
    public static final double hasNoteDebounceTime = 0.5; //s

    public static final double shootingVelocity = 10; //rot per seconds
    public static final double feedingVelocity = 10;

    //LOGPATH
    public static final String path = "ShooterSubsystem/";

    //Current Limits
    public static final double maxStatorCurrent = 80; //Amps
    public static final double maxSupplyCurrent = 60; //Amps
    public static final double maxSupplyTime= 0.1; //Seconds
    public static final boolean enableStatorCurrentLimit = true;
    public static final boolean enableSupplyCurrentLimit = true;

    //CONTROL LOOP GAINS
    public static final double kP = 0; 
    public static final double kI = 0; 
    public static final double kD = 0; 
    public static final double kV = 0.08; //voltage based on requested velocity
    public static final double kA = 0.01; //voltage based on requested acceleration
    public static final double kS = 0; //Voltage to overcome friction

    //PHYSICS
    public static final double momentOfInertia = GremlinUtil.lbIn2TokgM2(1.169 * 2); //pound inches^2, multiply by two bc two wheels per motor

    //GEARING
    public static final double gearing = 1.0 / 1.5; //1 motor rotation = 1.5 flywheel rotations

    //VELOCITY AND ACCELERATIOn
    public static final double maxVel = 10;
    public static final double maxAccel = 20;

    //Motor Configs
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(maxStatorCurrent)
        .withSupplyCurrentLimit(maxSupplyCurrent)
        .withSupplyTimeThreshold(maxSupplyTime)
        .withStatorCurrentLimitEnable(enableStatorCurrentLimit)
        .withSupplyCurrentLimitEnable(enableSupplyCurrentLimit);

    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs()
        .withSensorToMechanismRatio(gearing);

    public static final MotionMagicConfigs MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
        .withMotionMagicAcceleration(maxAccel)
        .withMotionMagicCruiseVelocity(maxVel);

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
        .withKP(kP)
        .withKI(kI)
        .withKD(kD)
        .withKV(kV)
        .withKA(kA)
        .withKS(kS);

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast);


    public static final TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
        .withFeedback(FEEDBACK_CONFIGS)
        .withMotionMagic(MOTION_MAGIC_CONFIGS)
        .withSlot0(SLOT0_CONFIGS)
        .withMotorOutput(MOTOR_OUTPUT_CONFIGS);
}