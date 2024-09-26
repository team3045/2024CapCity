// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.logging;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

/** Add your docs here. */
public class GremlinLogger extends DogLog{
    private static final String PID_KEY = "PID";

    public static void logTalonFX(String motorName, TalonFX motor){
        DogLog.log(motorName + "/DeviceID", motor.getDeviceID());

        DogLog.log(motorName + "/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        DogLog.log(motorName + "/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        DogLog.log(motorName + "/SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble());
        DogLog.log(motorName + "/OutputVoltage", motor.getMotorVoltage().getValueAsDouble());
        DogLog.log(motorName + "/Positon", motor.getPosition().getValueAsDouble());
        DogLog.log(motorName + "/RotorPosition", motor.getRotorPosition().getValueAsDouble());
        DogLog.log(motorName +"/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log(motorName + "/Velocity", motor.getVelocity().getValueAsDouble());
        DogLog.log(motorName + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
    }

    public static void logTalonFXPID(String motorName, TalonFX motor){
        DogLog.log(motorName + PID_KEY + "/Error", motor.getClosedLoopError().getValueAsDouble());
        DogLog.log(motorName + PID_KEY + "/TotalOutput", motor.getClosedLoopOutput().getValueAsDouble());
        DogLog.log(motorName + PID_KEY + "/Feedforward", motor.getClosedLoopFeedForward().getValueAsDouble());
        DogLog.log(motorName + PID_KEY + "/POutput", motor.getClosedLoopProportionalOutput().getValueAsDouble());
        DogLog.log(motorName + PID_KEY + "/Ioutput", motor.getClosedLoopIntegratedOutput().getValueAsDouble());
        DogLog.log(motorName + PID_KEY + "/DOutput", motor.getClosedLoopDerivativeOutput().getValueAsDouble());
        DogLog.log(motorName + PID_KEY + "/Reference", motor.getClosedLoopReference().getValueAsDouble());
    }

    public static void logStdDevs(String path, Vector<N3> stddevs){
        DogLog.log(path + "/Stddevs/XY", stddevs.getData()[0]);
        DogLog.log(path + "/Stddevs/Theta", stddevs.getData()[2]);
    }
}
