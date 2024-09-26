// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class GremlinLimelightCamera implements AutoCloseable{
    private static int InstanceCount = 0;
    public static final String kTableName = "";

    private final String name;
    private final Pose3d cameraPose;

    private double prevHeartbeatValue = -1;
    private double prevHeartbeatChangeTime = 0;
    private static final double HEARTBEAT_DEBOUNCE_SEC = 0.5;

    public GremlinLimelightCamera(String name, Pose3d cameraPose){
        this.cameraPose = cameraPose;
        this.name = name;

        HAL.report(tResourceType.kResourceType_PhotonCamera, InstanceCount);
        InstanceCount++;
    }

    /**
     * Returns limelight estimation of bot pose. Is always in blue origin coordinate 
     * system no matter alliance color. 
     * @return The botpose estimate using Megatag
     */
    public PoseEstimate getBotPoseEstimate(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    /**
     * Gets the limelight Megatag2 estimate of botpose. Is always in blue origin coordinate 
     * system no matter alliance color. 
     * @return The botpose estimate using Megatag
     */
    public PoseEstimate getBotPoseEstimateMT2(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    /**
     * Sets the robot orientation for use with limelight's MegaTag2 
     * @param rotation the current rotation of the robot, should be from your estimated pose
     * @param gyro the gyro used by robot for pose estimation, needed for angular rotation rates
     */
    public void setRobotHeading(double heading){
        //Megtag2 only cares about headin (yaw), dont need to worry about other values
        LimelightHelpers.SetRobotOrientation(
            name, 
            heading, 
            0, 
            0, 
            0,
            0,
            0);
    }

    /**
     * Use to focus only on tags that are both relevant and within tolerance, and filter out all other tags.
     * @param validIds tags to use for pose estimation
     */
    public void setValidIDsMT2(int[] validIds){
        LimelightHelpers.SetFiducialIDFiltersOverride(name, validIds);
    }

    /**
     * Requests camera to take a snapshot of the unprocessed scene. Snapshot is stored on 
     * limelight for future use. Calling it frequently will fill up disk space and 
     * eventually cause the system to stop working. Clear out snapshots in limelight UI 
     * periodically to prevent issues.
     */
    public void takeSnapshot(){
        LimelightHelpers.takeSnapshot(kTableName, name);
    }
    
    /**
     * Returns the active pipeline index.
     *
     * @return The active pipeline index.
     */
    public int getPipelineIndex(){
        return (int) LimelightHelpers.getCurrentPipelineIndex(name);
    }

    /**
     * Allows the user to select the active pipeline index.
     *
     * @param index The active pipeline index.
     */
    public void setPipelineIndex(int index){
        LimelightHelpers.setPipelineIndex(name, index);
    }

    /**
     * Returns the current LED mode.
     * 
     * @return The current LED mode.
     * {@code kOff (1)}: LEDs are forced off by code. {@code kBlink (2)}: LEDs are force to blink by code.
     * {@code kOn (3)}: LEDs are forced on by code. {@code kDefault(0)}: The LEDs will be controlled by Limelight 
     * pipeline settings, and not by robotcode.
     */
    public VisionLEDMode getLedMode(){
        switch ((int) LimelightHelpers.getLimelightNTDouble(name, "ledMode")) {
            case 1:
                return VisionLEDMode.kOff;
            case 2:
                return VisionLEDMode.kBlink;
            case 3: 
                return VisionLEDMode.kOn;
            case 0:
            default:
                return VisionLEDMode.kDefault;
        }
    }

    /**
     * Sets the LED mode.
     *
     * @param led The mode to set to.
     * {@code kOff (1)}: LEDs are forced off by code. {@code kBlink (2)}: LEDs are force to blink by code.
     * {@code kOn (3)}: LEDs are forced on by code. {@code kDefault(0)}: The LEDs will be controlled by Limelight 
     * pipeline settings, and not by robotcode.
     */
    public void setLED(VisionLEDMode led) {
        switch (led){
            case kBlink:
                LimelightHelpers.setLEDMode_ForceBlink(name);
                break;
            case kOff:
                LimelightHelpers.setLEDMode_ForceOff(name);
                break;
            case kOn:
                LimelightHelpers.setLEDMode_ForceOn(name);
                break;
            case kDefault:
            default:
                LimelightHelpers.setLEDMode_PipelineControl(name);
                break;
        }
    }

    /**
     * Returns the name of the camera. This will return the same value that was given to the
     * constructor as cameraName.
     *
     * @return The name of the camera.
     */
    public String getName() {
        return name;
    }

    /**
     * Get the 3d position of camera on robot. Likely not often called 
     * as the camera pose should be configured in limelight UI. 
     * @return The 3d position of the the camera with robot center as origin
     */
    public Pose3d getCameraPose(){
        return cameraPose;
    }

    /**
     * Returns whether the camera is connected and actively returning new data. Connection status is
     * debounced. 
     *
     * @return True if the camera is actively sending frame data, false otherwise.
     */
    public boolean isConnected() {
        var curHeartbeat = LimelightHelpers.getLimelightNTDouble(name, "hb");
        var now = Timer.getFPGATimestamp();

        //If new heartbeat then will return true
        if (curHeartbeat != prevHeartbeatValue) {
            // New heartbeat value from the coprocessor
            prevHeartbeatChangeTime = now;
            prevHeartbeatValue = curHeartbeat;
        }

        //if not new heartbeat make sure that its been atleast the debounce time, 
        //if its less then the deboucne time limelight hasnt had enough time to increment heartbeat yet
        return (now - prevHeartbeatChangeTime) < HEARTBEAT_DEBOUNCE_SEC;
    }

    @Override
    public void close() throws Exception {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'close'");
    }
    
}
