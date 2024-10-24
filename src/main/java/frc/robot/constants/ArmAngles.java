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
    public static final ArrayList<Double> speakerTags = new ArrayList<>(Arrays.asList(3.0,7.0));

    public static void initLookuptable(){
        map.put(1.3,ArmConstants.defaultShotAngle);
        map.put(1.59, -33.0);
        map.put(1.8334, -37.0);
        map.put(2.0874, -40.0);
        map.put(2.3414, -44.0);
        map.put(2.5954, -46.0);
        map.put(2.8494, -49.0);
        map.put(3.1034, -51.0);
        map.put(3.3574, -54.0);
        map.put(4.094, -58.0);
        map.put(4.3414, -60.0);
        map.put(4.5954, -62.0);
        map.put(5.1034, -64.0);
    }
}
