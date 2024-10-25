// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class GremlinLogger extends DogLog {
    private static final String PID_KEY = "PID";

    public static void logTalonFX(String motorName, TalonFX motor) {
        GremlinLogger.logSD(motorName + "/DeviceID", motor.getDeviceID());
        GremlinLogger.logSD(motorName + "/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/OutputVoltage", motor.getMotorVoltage().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/Positon", motor.getPosition().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/RotorPosition", motor.getRotorPosition().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/Velocity", motor.getVelocity().getValueAsDouble());
        GremlinLogger.logSD(motorName + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
    }

    public static void logTalonFXPID(String motorName, TalonFX motor) {
        GremlinLogger.logSD(motorName + PID_KEY + "/Error", motor.getClosedLoopError().getValueAsDouble());
        GremlinLogger.logSD(motorName + PID_KEY + "/TotalOutput", motor.getClosedLoopOutput().getValueAsDouble());
        GremlinLogger.logSD(motorName + PID_KEY + "/Feedforward", motor.getClosedLoopFeedForward().getValueAsDouble());
        GremlinLogger.logSD(motorName + PID_KEY + "/POutput",
                motor.getClosedLoopProportionalOutput().getValueAsDouble());
        GremlinLogger.logSD(motorName + PID_KEY + "/Ioutput", motor.getClosedLoopIntegratedOutput().getValueAsDouble());
        GremlinLogger.logSD(motorName + PID_KEY + "/DOutput", motor.getClosedLoopDerivativeOutput().getValueAsDouble());
        GremlinLogger.logSD(motorName + PID_KEY + "/Reference", motor.getClosedLoopReference().getValueAsDouble());
    }

    public static void logStdDevs(String path, Vector<N3> stddevs) {
        GremlinLogger.logSD(path + "/Stddevs/XY", stddevs.getData()[0]);
        GremlinLogger.logSD(path + "/Stddevs/Theta", stddevs.getData()[2]);
    }

    /**
     * Logs a double and also puts it on Smartdashboard
     * 
     * @param key
     * @param value
     */
    public static void logSD(String key, double value) {
        log(key, value);
        SmartDashboard.putNumber(key, value);
    }

    /**
     * Logs a String and also puts it on Smartdashboard
     * 
     * @param key
     * @param value
     */
    public static void logSD(String key, String value) {
        log(key, value);
        SmartDashboard.putString(key, value);
    }

    /**
     * Logs a boolen and also puts it on Smartdashboard
     * 
     * @param key
     * @param value
     */
    public static void logSD(String key, boolean value) {
        log(key, value);
        SmartDashboard.putBoolean(key, value);
    }

    /**
     * Logs a double array and also puts it on Smartdashboard
     * 
     * @param key
     * @param value
     */
    public static void logSD(String key, double[] value) {
        log(key, value);
        SmartDashboard.putNumberArray(key, value);
    }

    /**
     * Logs a Pose3d and also puts it on Smartdashboard
     * 
     * @param key
     * @param value
     */
    public static void logSD(String key, Pose3d value) {
        log(key, value);
        SmartDashboard.putNumberArray(key, 
            new double[]{
                value.getX(),value.getY(),value.getZ(),
                value.getRotation().getX(),value.getRotation().getY(),value.getRotation().getZ()
            }
        );
    }

    /**Logs a Transform3d and also puts it on Smartdashboard
     * @param key
     * @param value
     */
    public static void logSD(String key, Transform3d value){
        Pose3d pose = GeomUtil.transform3dToPose3d(value);
        logSD(key, pose);
    }

    /**Logs a Pose2d and also puts it on Smartdashboard
     * @param key
     * @param value
     */ // TODO: not sure if a 3 element array is correct
    public static void logSD(String key, Pose2d value) {
        log(key, value);
        SmartDashboard.putNumberArray(key,
                new double[] {
                        value.getX(),
                        value.getY(),
                        value.getRotation().getRadians()
                });
    }

    /**
     * Logs a double and also puts it on Smartdashboard
     * 
     * @param key
     * @param value
     */
    public static void logSD(String key, int value) {
        log(key, value);
        SmartDashboard.putNumber(key, value);
    }
}
