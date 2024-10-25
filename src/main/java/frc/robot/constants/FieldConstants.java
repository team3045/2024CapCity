package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class FieldConstants {

  public static final boolean isShopField = false;

  public static final double fieldLength = 16.451;
  public static final double fieldWidth = 8.211;

    public static final double GOAL_INWARD_SHIFT = 0.0;

    // AprilTag constants
    public static final double aprilTagWidth = Units.inchesToMeters(8.12500);

  public static final Translation2d ampCenter =
      new Translation2d(Units.inchesToMeters(72.455), fieldWidth);

  public static final Rotation2d ampAngle = Rotation2d.fromDegrees(90);

  public static final Translation2d passingTarget = new Translation2d(0.81, 7.31);
  public static final Translation2d lowPassingTarget = new Translation2d(2.13, 8.07);

    public static final double noteSpacing = Units.inchesToMeters(64);

    public static final List<Translation2d> notePositions = List.of(
            new Translation2d(fieldLength / 2, Units.inchesToMeters(32)),
            new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing),
            new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing * 2),
            new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing * 3),
            new Translation2d(fieldLength / 2, Units.inchesToMeters(32) + noteSpacing * 4));

    public static final AprilTagFieldLayout aprilTags = new AprilTagFieldLayout(
            List.of(
                    new AprilTag(
                            1,
                            new Pose3d(
                                    15.079471999999997,
                                    0.24587199999999998,
                                    1.355852,
                                    new Rotation3d(
                                            new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
                    new AprilTag(
                            2,
                            new Pose3d(
                                    16.185134,
                                    0.883666,
                                    1.355852,
                                    new Rotation3d(
                                            new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
                    new AprilTag(
                            3,
                            new Pose3d(
                                    16.579342,
                                    4.982717999999999,
                                    1.4511020000000001,
                                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
                    new AprilTag(
                            4,
                            new Pose3d(
                                    16.579342,
                                    5.547867999999999,
                                    1.4511020000000001,
                                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
                    new AprilTag(
                            5,
                            new Pose3d(
                                    14.700757999999999,
                                    8.2042,
                                    1.355852,
                                    new Rotation3d(
                                            new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
                    new AprilTag(
                            6,
                            new Pose3d(
                                    1.8415,
                                    8.2042,
                                    1.355852,
                                    new Rotation3d(
                                            new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476)))),
                    new AprilTag(
                            7,
                            new Pose3d(
                                    -0.038099999999999995,
                                    5.547867999999999,
                                    1.4511020000000001,
                                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
                    new AprilTag(
                            8,
                            new Pose3d(
                                    -0.038099999999999995,
                                    4.982717999999999,
                                    1.4511020000000001,
                                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
                    new AprilTag(
                            9,
                            new Pose3d(
                                    0.356108,
                                    0.883666,
                                    1.355852,
                                    new Rotation3d(
                                            new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
                    new AprilTag(
                            10,
                            new Pose3d(
                                    1.4615159999999998,
                                    0.24587199999999998,
                                    1.355852,
                                    new Rotation3d(
                                            new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
                    new AprilTag(
                            11,
                            new Pose3d(
                                    11.904726,
                                    3.7132259999999997,
                                    1.3208,
                                    new Rotation3d(
                                            new Quaternion(-0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
                    new AprilTag(
                            12,
                            new Pose3d(
                                    11.904726,
                                    4.49834,
                                    1.3208,
                                    new Rotation3d(
                                            new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
                    new AprilTag(
                            13,
                            new Pose3d(
                                    11.220196,
                                    4.105148,
                                    1.3208,
                                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
                    new AprilTag(
                            14,
                            new Pose3d(
                                    5.320792,
                                    4.105148,
                                    1.3208,
                                    new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
                    new AprilTag(
                            15,
                            new Pose3d(
                                    4.641342,
                                    4.49834,
                                    1.3208,
                                    new Rotation3d(
                                            new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
                    new AprilTag(
                            16,
                            new Pose3d(
                                    4.641342,
                                    3.7132259999999997,
                                    1.3208,
                                    new Rotation3d(
                                            new Quaternion(-0.4999999999999998, 0.0, 0.0, 0.8660254037844387))))),
            fieldLength,
            fieldWidth);


    public static final double speakerToWall = 6.64765625; //m
    public static final double speakerX = -0.038099999999999995;
    public static final double speakerY = 5.547867999999999;
    public static final double speakerZ = 1.4511020000000001;

    public static final double shop7Y = 2.3796625;
    public static final double shop7Z = 1.36921875;
    public static final double shop6Y = 1.3001625;
    public static final double shop6Z = 1.5859125;
    public static final double shop2Y = 3.819525;
    public static final double shop2Z = 2.3320375;
    public static final double wallOffset = -0.00635;
    public static final double tempX = speakerX + speakerToWall + wallOffset;
    public static final double tempY = speakerY + (shop7Y-shop2Y);
    public static final double tempZ = speakerZ + (shop6Z - shop7Z);
    public static final AprilTagFieldLayout shopLayout = new AprilTagFieldLayout(
        List.of(
            new AprilTag(
                  7,
                  new Pose3d(
                      speakerX,
                      speakerY,
                      speakerZ,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                  8,
                  new Pose3d(
                      speakerX,
                      4.982717999999999,
                      speakerZ,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(
                6,
                new Pose3d(
                    speakerX + speakerToWall + wallOffset, 
                    speakerY + (shop7Y-shop6Y), 
                    speakerZ + (shop6Z - shop7Z), 
                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
            new AprilTag(
                2,
                new Pose3d(
                    speakerX + speakerToWall + wallOffset, 
                    speakerY + (shop7Y-shop2Y), 
                    speakerZ + (shop2Z - shop7Z), 
                    new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
            new AprilTag(
                  1,
                  new Pose3d(
                      0,
                      0,
                      0,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, 0.0, 0.0, 0.7071067811865476))))
        ),
        fieldLength,
        fieldWidth
    );

  // Target Rectangle
  public static final Pose3d tagSevenPose = aprilTags.getTagPose(7).get();
  public static final Pose3d tagThreePose = aprilTags.getTagPose(3).get();

    // public static final Translation3d[] targetVerticies = {
    // new Translation3d(
    // tagSevenPose.getX(),
    // tagSevenPose.getY() - Units.inchesToMeters(21.125),
    // tagSevenPose.getZ() + Units.inchesToMeters(21.125)),
    // new Translation3d(
    // tagSevenPose.getX(),
    // tagSevenPose.getY() + Units.inchesToMeters(21.125),
    // tagSevenPose.getZ() + Units.inchesToMeters(21.125)),
    // new Translation3d(
    // tagSevenPose.getX() + Units.inchesToMeters(20.0),
    // tagSevenPose.getY() - Units.inchesToMeters(21.25),
    // tagSevenPose.getZ() + Units.inchesToMeters(26.35)),
    // new Translation3d(
    // tagSevenPose.getX() + Units.inchesToMeters(20.0),
    // tagSevenPose.getY() + Units.inchesToMeters(21.125),
    // tagSevenPose.getZ() + Units.inchesToMeters(26.35))
    // };

    public static final Translation3d[] targetVerticies = {
            new Translation3d(
                    tagSevenPose.getX() - GOAL_INWARD_SHIFT,
                    tagSevenPose.getY() - Units.inchesToMeters(21.125),
                    tagSevenPose.getZ() + Units.inchesToMeters(21.125)),
            new Translation3d(
                    tagSevenPose.getX() - GOAL_INWARD_SHIFT,
                    tagSevenPose.getY() + Units.inchesToMeters(21.125),
                    tagSevenPose.getZ() + Units.inchesToMeters(21.125)),
            new Translation3d(
                    tagSevenPose.getX() + Units.inchesToMeters(20.0) - GOAL_INWARD_SHIFT,
                    tagSevenPose.getY() - Units.inchesToMeters(21.25),
                    tagSevenPose.getZ() + Units.inchesToMeters(26.35)),
            new Translation3d(
                    tagSevenPose.getX() + Units.inchesToMeters(20.0) - GOAL_INWARD_SHIFT,
                    tagSevenPose.getY() + Units.inchesToMeters(21.125),
                    tagSevenPose.getZ() + Units.inchesToMeters(26.35))
    };

    public static final Pose2d targetPoseBlue = tagSevenPose.toPose2d();
    public static final Pose2d targetPoseRed = tagThreePose.toPose2d();

    public static final double targetWidth = Units.inchesToMeters(42);
}
