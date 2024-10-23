// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class DriveConstants {
    public static final double rangeTheshold = 6; //3 meters
    public static final double TrueMaxAngularRate = 1.5 * Math.PI;
    public static final double TrueMaxSpeed = TunerConstants.kSpeedAt12VoltsMps;

    public static final double headingP = 1;
    public static final double headingI = 0;
    public static final double headingD = 0;

    public static final double XYDeadband = 0.1;
    public static final double RotationDeadband = 0.1;

    public static double appliedMaxSpeed = TrueMaxSpeed;
    public static double appliedMaxAngularRate = TrueMaxAngularRate;
}
