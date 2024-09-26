// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

/** Add your docs here. */
public class GremlinUnits {
    //private constructor so cant be instantiated
    private GremlinUnits(){}

    /*
     * Converts from pound inches squared to kilogram meters squared.
     * These units are for moment of Inertia. 
     */
    public static double lbIn2TokgM2(double lbIn2){
        return lbIn2 * 0.000293;
    }

}
