// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;
import frc.robot.commons.GremlinUtil;

/** Add your docs here. */
public class ArmConstants {
    // CANBUS
    public static final int leftMotorID = 13;
    public static final int rightMotorID = 14;
    public static final String canbus = "3045 Canivore";
    public static final int cancoderID = 20;

    // PHYSICS
    public static final double armMOI = GremlinUtil.lbIn2TokgM2(3490); // in^2 lb
    public static final double armCOM = Units.inchesToMeters(15.044); // Close enough to directly below pivot for our
                                                                      // purposes
    public static final double armMass = Units.lbsToKilograms(17.044);
    public static final double mech2dOffset = 180; // degrees
    public static final double simPositionX = -0.051;
    public static final double simPositionY = 0;
    public static final double simPositionZ = 0.6096;

    // ANGLES
    public static final double minAngle = 18.26 - 90; // degrees
    public static final double maxAngle = 110;
    public static final double intakeAngle = minAngle;
    public static final double ampAngle = 47;
    public static final double defaultShotAngle = -28;
    public static final double passAngle = -45;

    // TOLERANCES
    public static final double angleTolerance = 0.5;
    public static final double velocityTolerance = 1; // deg / s
    public static final double atTargetDelay = 0.1; // sec

    // LOGPATH
    public static final String path = "ArmSubsystem/";

    // MECHANISM
    public static final double canvasWidth = Units.inchesToMeters(32); // Technically arbitray units but we'll use
                                                                       // meters to be conssistent
    public static final double canvasHeight = Units.inchesToMeters(32);
    public static final double rootX = canvasWidth / 2;
    public static final double rootY = Units.inchesToMeters(25.752224);
    public static final double simX = 0;
    public static final double simY = 0;
    public static final double simZ = 0;

    // CONFIGURATION

    // Current Limits
    public static final double maxStatorCurrent = 120; // Amps
    public static final double maxSupplyCurrent = 60; // Amps
    public static final double maxSupplyTime = 0.1; // Seconds
    public static final boolean enableStatorCurrentLimit = true;
    public static final boolean enableSupplyCurrentLimit = true;

    // Feedback Configs
    public static final double rotorToSensorRatio = (60.0 / 12.0) * (60.0 / 18.0); // gearing from motor to cancoder
    public static final double sensorToMechanismRatio = (48.0 / 16.0); // gearing from cancoder to arm pivot
    public static final double totalGearing = rotorToSensorRatio * sensorToMechanismRatio;

    // Inverts
    public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;

    //Velocity and Acceleration
    public static final double targetAcceleration = 2; //Rot per Sec^2, This is mostly going to be your max workable acceleration
    public static final double targetVelocity = 1; //Rot per Sec, This is mostly going to be max workable Velocity

    // Control Loop Gains
    public static final GravityTypeValue gravity = GravityTypeValue.Elevator_Static;
    public static final double kP = 500;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double kG = 0.701825; // voltage required to overcome gravity
    public static final double kV = 1.593; // voltage based on requested velocity
    public static final double kA = 0.78869; // voltage based on requested acceleration
    public static final double kS = 0.29658; // Voltage to overcome friction

    // Cancoder Settings
    public static final double angleOffset = -0.177; // rotations, divide b/c arm to cancocer
    public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.Clockwise_Positive;

    // Motor Configs
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(maxStatorCurrent)
            .withSupplyCurrentLimit(maxSupplyCurrent)
            .withSupplyTimeThreshold(maxSupplyTime)
            .withStatorCurrentLimitEnable(enableStatorCurrentLimit)
            .withSupplyCurrentLimitEnable(enableSupplyCurrentLimit);

    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(cancoderID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withSensorToMechanismRatio(sensorToMechanismRatio).withFeedbackRotorOffset(0)
            .withRotorToSensorRatio(rotorToSensorRatio);

    public static final MotionMagicConfigs motionMagic = new MotionMagicConfigs()
            .withMotionMagicAcceleration(targetAcceleration)
            .withMotionMagicCruiseVelocity(targetVelocity);

    public static final Slot0Configs slot0 = new Slot0Configs()
            .withGravityType(gravity)
            .withKP(kP)
            .withKI(kI)
            .withKP(kP)
            .withKG(0)
            .withKV(kV)
            .withKS(kS)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    // Apply Motor Invert in Armsubsystem b/c left and right are opposite
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(currentLimits)
            .withFeedback(feedbackConfigs)
            .withMotionMagic(motionMagic)
            .withSlot0(slot0);

    // Cancoder Configs
    public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                    .withMagnetOffset(angleOffset)
                    .withSensorDirection(cancoderInvert));
}
