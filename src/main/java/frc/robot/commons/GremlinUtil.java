// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

/** Add your docs here. */
public class GremlinUtil {
    // private constructor so cant be instantiated
    private GremlinUtil() {
    }

    /**
     * Converts from pound inches squared to kilogram meters squared.
     * These units are for moment of Inertia.
     * 
     * @param lnIn2 Moment of inertia in pound inches squared
     * @return Moment of inertia in kilogram metre squared
     */
    public static double lbIn2TokgM2(double lbIn2) {
        return lbIn2 * 0.000293;
    }

    /**
     * Get the position for example of a motor rotor from an arm angle.
     * Or do same for velocity
     * 
     * @param gearing    The gear ratio in the form rotationsPlant / rotationsMotor
     * @param plantAngle The angle of the plant ie. the arm if controlling an arm
     *                   system
     * @return returns whatever the value, position, velocity. etc, is after gearing
     *         is taken into account
     */
    public static double valueAfterGearing(double plantValue, double gearing) {
        return plantValue / gearing;
    }

    public static double average(Double... doubles) {
        double total = 0;
        for (double x : doubles)
            total += x;

        return total / doubles.length;
    }

    /**
     * Clamp a value between a max and min, logging a fault if it is outside of the
     * desired range.
     * 
     * @param max   the maximum the value should be
     * @param min   the minimum the value should be
     * @param value the value you want clamped
     * 
     * @return the value clamps between max and min
     */
    public static double clampWithLogs(double max, double min, double value) {
        if (value > max) {
            GremlinLogger.logFault("Value Exceeds Max Angle");
            return max;
        } else if (value < min) {
            GremlinLogger.logFault("Value is lower than Min angle");
            return min;
        } else {
            return value;
        }
    }

}
