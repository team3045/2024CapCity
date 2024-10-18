// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ArmAngles {
    public static final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
    public static final Pose3d speakerAimingPoint = new Pose3d();
    public static final ArrayList<Double> speakerTags = new ArrayList<>(Arrays.asList(3.0, 7.0));

    public static void initLookuptable() {
        map.put(0.0, 155.0);
        map.put(0.25, 155.0);
        map.put(0.5, 151.0);
        map.put(0.75, 148.0);
        map.put(1.0, 145.0);
        map.put(1.25, 142.0);
        map.put(1.5, 139.0);
        map.put(1.75, 136.0);
        map.put(2.0, 133.0);
        map.put(2.25, 132.0);
        map.put(2.5, 30.0);
        map.put(2.75, 40.0);
        map.put(3.0, 50.0);
        map.put(3.25, 60.0);
        map.put(3.5, 70.0);
        map.put(3.75, 80.0);
        map.put(4.00, 90.0);
        map.put(4.25, 100.0);
        map.put(4.5, 110.0);
        map.put(4.75, 120.0);
        map.put(5.0, 130.0);
        map.put(5.25, 140.0);
        map.put(5.5, 150.0);
        map.put(5.75, 160.0);
        map.put(6.0, 170.0);
    }
}
