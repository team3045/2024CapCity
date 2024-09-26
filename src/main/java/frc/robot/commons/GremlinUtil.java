// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

/** Add your docs here. */
public class GremlinUtil {
    //private constructor so cant be instantiated
    private GremlinUtil(){}

    /*
     * Converts from pound inches squared to kilogram meters squared.
     * These units are for moment of Inertia. 
     */
    public static double lbIn2TokgM2(double lbIn2){
        return lbIn2 * 0.000293;
    }

    /*
     * Get the position for example of a motor rotor from an arm angle. 
     * Or do same for velocity
     * 
     * @param gearing The gear ratio in the form rotationsPlant / rotationsMotor
     * @param plantAngle The angle of the plant ie. the arm if controlling an arm system
     */
    public static double valueAfterGearing(double plantValue, double gearing){
        return plantValue / gearing;
    }

}
