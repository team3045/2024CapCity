// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.HashSet;
import java.util.Set;

import frc.robot.commons.PolynomialRegression;

/** Add your docs here. */
public class VisionConstants {
    //private constructor so cant be instantiated
    private VisionConstants(){}

    public static final String CAMERA_LOG_PATH = "VISION/CAMERAS/";

    public static final Set<Integer> EXCLUDED_TAG_IDS = new HashSet<>();

    public static final double MAX_AMBIGUITY = 0.15;
    public static final double FIELD_BORDER_MARGIN = 0.5;

    public static final PolynomialRegression XY_STDDEV_MODEL =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38},
          2);
    public static final PolynomialRegression THETA_STDDEV_MODEL =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);
      
}
