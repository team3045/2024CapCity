// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.security.CryptoPrimitive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** Add your docs here. */
public class ArmConstants {

    //Canbus
    public static final int leftMotorID = 1;
    public static final int rightMotorID = 2;
    public static final String canbus = "rio";
    public static final int cancoderID = 3;


    //CONFIGURATION

    //Current Limits
    public static final double maxStatorCurrent = 120; //Amps
    public static final double maxSupplyCurrent = 60; //Amps
    public static final double maxSupplyTime= 0.1; //Seconds
    public static final boolean enableStatorCurrentLimit = true;
    public static final boolean enableSupplyCurrentLimit = true;

    //Feedback Configs
    public static final double rotorToSensorRatio = 1; //gearing from motor to cancoder
    public static final double sensorToMechanismRatio = 1; //gearing from cancoder to arm pivot

    //Inverts
    public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;

    //Velocity and Acceleration
    public static final double targetAcceleration = 1; //Rot per Sec^2, This is mostly going to be your max workable acceleration
    public static final double targetVelocity = 1; //Rot per Sec, This is mostly going to be max workable Velocity

    //Control Loop Gains
    public static final GravityTypeValue gravity = GravityTypeValue.Arm_Cosine; //scale kG based on the arm angle
    public static final double kP = 1; 
    public static final double kI = 0; 
    public static final double kD = 0; 
    public static final double kG = 0; //voltage required to overcome gravity
    public static final double kV = 0; //voltage based on requested velocity
    public static final double kA = 0; //voltage based on requested acceleration
    public static final double kS = 0; //Voltage to overcome friction


    
    //Motor Configs
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(maxStatorCurrent)
        .withSupplyCurrentLimit(maxSupplyCurrent)
        .withSupplyTimeThreshold(maxSupplyTime)
        .withStatorCurrentLimitEnable(enableStatorCurrentLimit)
        .withSupplyCurrentLimitEnable(enableSupplyCurrentLimit);
    
    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFusedCANcoder(new CoreCANcoder(cancoderID))
        .withRotorToSensorRatio(rotorToSensorRatio)
        .withSensorToMechanismRatio(sensorToMechanismRatio);

    public static final MotionMagicConfigs motionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(cancoderID)
        .withMotionMagicCruiseVelocity(cancoderID);

    public static final Slot0Configs slot0 = new Slot0Configs()
        .withGravityType(gravity)
        .withKP(kP)
        .withKI(kI)
        .withKP(kP)
        .withKG(kG)
        .withKV(kV)
        .withKS(kS)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    //Apply Motor Invert in Armsubsystem b/c left and right are opposite
    public static final TalonFXConfiguration config = new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withFeedback(feedbackConfigs)
        .withMotionMagic(motionMagic)
        .withSlot0(slot0);
}
