// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
}
