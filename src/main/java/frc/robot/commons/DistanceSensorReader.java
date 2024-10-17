// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

import java.util.concurrent.atomic.AtomicReference;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/** Add your docs here. */
public class DistanceSensorReader implements Runnable{
    private int id;
    private final TimeOfFlight rangeSensor;

    /*For Thread Safety */
    private AtomicReference<Double> rangeSensorValue = new AtomicReference<>();
    private AtomicReference<Boolean> validMeasurment = new AtomicReference<>();

    public DistanceSensorReader(int id){
        this.id = id;
        rangeSensor = new TimeOfFlight(this.id);
        rangeSensor.setRangingMode(RangingMode.Short, 50);
    }

    @Override
    public void run() {
        rangeSensorValue.set(rangeSensor.getRange());
        validMeasurment.set(rangeSensor.isRangeValid());
    }

    public double getRange(){
        return rangeSensorValue.get();
    }

    public boolean isValid(){
        return validMeasurment.get();
    }

}
