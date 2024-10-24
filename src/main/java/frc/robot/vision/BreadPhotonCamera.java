// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonVersion;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.common.networktables.PacketSubscriber;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

/** Add your docs here. */
public class BreadPhotonCamera implements AutoCloseable {
    private static int InstanceCount = 0;
    public static final String kTableName = "photonvision";

    private final String path;
    private final String name;
    private final Pose3d camPose;
    private final PhotonCamera photonCamera;

    private final NetworkTable cameraTable;
    public PacketSubscriber<PhotonPipelineResult> resultSubscriber;
    BooleanPublisher driverModePublisher;
    BooleanSubscriber driverModeSubscriber;
    DoublePublisher latencyMillisEntry;
    BooleanPublisher hasTargetEntry;
    DoublePublisher targetPitchEntry;
    DoublePublisher targetYawEntry;
    DoublePublisher targetAreaEntry;
    DoubleArrayPublisher targetPoseEntry;
    DoublePublisher targetSkewEntry;
    StringSubscriber versionEntry;
    IntegerEntry inputSaveImgEntry, outputSaveImgEntry;
    IntegerPublisher pipelineIndexRequest, ledModeRequest;
    IntegerSubscriber pipelineIndexState, ledModeState;
    IntegerSubscriber heartbeatEntry;
    DoubleArraySubscriber cameraIntrinsicsSubscriber;
    DoubleArraySubscriber cameraDistortionSubscriber;

    private static final boolean VERSION_CHECK_ENABLED = true;
    private static long VERSION_CHECK_INTERVAL = 5;
    private double lastVersionCheckTime = 0;

    private long prevHeartbeatValue = -1;
    private double prevHeartbeatChangeTime = 0;
    private static final double HEARTBEAT_DEBOUNCE_SEC = 0.5;

    public BreadPhotonCamera(NetworkTableInstance instance, String cameraName, Pose3d camPose) {
        name = cameraName;
        this.camPose = camPose;
        var photonvision_root_table = instance.getTable(kTableName);
        this.cameraTable = photonvision_root_table.getSubTable(cameraName);
        path = cameraTable.getPath();

        photonCamera = new PhotonCamera(cameraName);

        var rawBytesEntry = cameraTable
                .getRawTopic("rawBytes")
                .subscribe(
                        "rawBytes", new byte[] {}, PubSubOption.periodic(0.01), PubSubOption.sendAll(true));
        resultSubscriber = new PacketSubscriber<>(
                rawBytesEntry, PhotonPipelineResult.serde, new PhotonPipelineResult());
        driverModePublisher = cameraTable.getBooleanTopic("driverModeRequest").publish();
        driverModeSubscriber = cameraTable.getBooleanTopic("driverMode").subscribe(false);
        inputSaveImgEntry = cameraTable.getIntegerTopic("inputSaveImgCmd").getEntry(0);
        outputSaveImgEntry = cameraTable.getIntegerTopic("outputSaveImgCmd").getEntry(0);
        pipelineIndexRequest = cameraTable.getIntegerTopic("pipelineIndexRequest").publish();
        pipelineIndexState = cameraTable.getIntegerTopic("pipelineIndexState").subscribe(0);
        heartbeatEntry = cameraTable.getIntegerTopic("heartbeat").subscribe(-1);
        cameraIntrinsicsSubscriber = cameraTable.getDoubleArrayTopic("cameraIntrinsics").subscribe(null);
        cameraDistortionSubscriber = cameraTable.getDoubleArrayTopic("cameraDistortion").subscribe(null);

        ledModeRequest = photonvision_root_table.getIntegerTopic("ledModeRequest").publish();
        ledModeState = photonvision_root_table.getIntegerTopic("ledModeState").subscribe(-1);
        versionEntry = photonvision_root_table.getStringTopic("version").subscribe("");

        // Existing is enough to make this multisubscriber do its thing
        @SuppressWarnings({ "unused", "resource" })
        MultiSubscriber m_topicNameSubscriber = new MultiSubscriber(
                instance, new String[] { "/photonvision/" }, PubSubOption.topicsOnly(true));

        HAL.report(tResourceType.kResourceType_PhotonCamera, InstanceCount);
        InstanceCount++;
    }

    /**
     * Returns the latest pipeline result.
     *
     * @return The latest pipeline result.
     */
    public PhotonPipelineResult getLatestResult() {
        verifyVersion();

        // Return result.
        return resultSubscriber.get();
    }

    /**
     * Returns whether the camera is in driver mode.
     *
     * @return Whether the camera is in driver mode.
     */
    public boolean getDriverMode() {
        return driverModeSubscriber.get();
    }

    /**
     * Toggles driver mode.
     *
     * @param driverMode Whether to set driver mode.
     */
    public void setDriverMode(boolean driverMode) {
        driverModePublisher.set(driverMode);
    }

    /**
     * Request the camera to save a new image file from the input camera stream with
     * overlays. Images
     * take up space in the filesystem of the PhotonCamera. Calling it frequently
     * will fill up disk
     * space and eventually cause the system to stop working. Clear out images in
     * /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues.
     */
    public void takeInputSnapshot() {
        inputSaveImgEntry.set(inputSaveImgEntry.get() + 1);
    }

    /**
     * Request the camera to save a new image file from the output stream with
     * overlays. Images take
     * up space in the filesystem of the PhotonCamera. Calling it frequently will
     * fill up disk space
     * and eventually cause the system to stop working. Clear out images in
     * /opt/photonvision/photonvision_config/imgSaves frequently to prevent issues.
     */
    public void takeOutputSnapshot() {
        outputSaveImgEntry.set(outputSaveImgEntry.get() + 1);
    }

    /**
     * Returns the active pipeline index.
     *
     * @return The active pipeline index.
     */
    public int getPipelineIndex() {
        return (int) pipelineIndexState.get(0);
    }

    /**
     * Allows the user to select the active pipeline index.
     *
     * @param index The active pipeline index.
     */
    public void setPipelineIndex(int index) {
        pipelineIndexRequest.set(index);
    }

    /**
     * Returns the current LED mode.
     *
     * @return The current LED mode.
     */
    public VisionLEDMode getLEDMode() {
        int value = (int) ledModeState.get(-1);
        switch (value) {
            case 0:
                return VisionLEDMode.kOff;
            case 1:
                return VisionLEDMode.kOn;
            case 2:
                return VisionLEDMode.kBlink;
            case -1:
            default:
                return VisionLEDMode.kDefault;
        }
    }

    /**
     * Sets the LED mode.
     *
     * @param led The mode to set to.
     */
    public void setLED(VisionLEDMode led) {
        ledModeRequest.set(led.value);
    }

    /**
     * Returns whether the latest target result has targets.
     *
     * <p>
     * This method is deprecated; {@link PhotonPipelineResult#hasTargets()} should
     * be used instead.
     *
     * @deprecated This method should be replaced with
     *             {@link PhotonPipelineResult#hasTargets()}
     * @return Whether the latest target result has targets.
     */
    @Deprecated
    public boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    /**
     * Returns the name of the camera. This will return the same value that was
     * given to the
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
     * 
     * @return The 3d position of the the camera with robot center as origin
     */
    public Pose3d getCameraPose() {
        return camPose;
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    /**
     * Returns whether the camera is connected and actively returning new data.
     * Connection status is
     * debounced.
     *
     * @return True if the camera is actively sending frame data, false otherwise.
     */
    public boolean isConnected() {
        var curHeartbeat = heartbeatEntry.get();
        var now = Timer.getFPGATimestamp();

        if (curHeartbeat != prevHeartbeatValue) {
            // New heartbeat value from the coprocessor
            prevHeartbeatChangeTime = now;
            prevHeartbeatValue = curHeartbeat;
        }

        return (now - prevHeartbeatChangeTime) < HEARTBEAT_DEBOUNCE_SEC;
    }

    public Optional<Matrix<N3, N3>> getCameraMatrix() {
        var cameraMatrix = cameraIntrinsicsSubscriber.get();
        if (cameraMatrix != null && cameraMatrix.length == 9) {
            return Optional.of(MatBuilder.fill(Nat.N3(), Nat.N3(), cameraMatrix));
        } else
            return Optional.empty();
    }

    public Optional<Matrix<N5, N1>> getDistCoeffs() {
        var distCoeffs = cameraDistortionSubscriber.get();
        if (distCoeffs != null && distCoeffs.length == 5) {
            return Optional.of(MatBuilder.fill(Nat.N5(), Nat.N1(), distCoeffs));
        } else
            return Optional.empty();
    }

    /**
     * Gets the NetworkTable representing this camera's subtable. You probably don't
     * ever need to call
     * this.
     */
    public final NetworkTable getCameraTable() {
        return cameraTable;
    }

    @Override
    public void close() throws Exception {
        resultSubscriber.close();
        driverModePublisher.close();
        driverModeSubscriber.close();
        latencyMillisEntry.close();
        hasTargetEntry.close();
        targetPitchEntry.close();
        targetYawEntry.close();
        targetAreaEntry.close();
        targetPoseEntry.close();
        targetSkewEntry.close();
        versionEntry.close();
        inputSaveImgEntry.close();
        outputSaveImgEntry.close();
        pipelineIndexRequest.close();
        pipelineIndexState.close();
        ledModeRequest.close();
        ledModeState.close();
        pipelineIndexRequest.close();
        cameraIntrinsicsSubscriber.close();
        cameraDistortionSubscriber.close();
    }

    private void verifyVersion() {
        if (!VERSION_CHECK_ENABLED)
            return;

        if ((Timer.getFPGATimestamp() - lastVersionCheckTime) < VERSION_CHECK_INTERVAL)
            return;
        lastVersionCheckTime = Timer.getFPGATimestamp();

        // Heartbeat entry is assumed to always be present. If it's not present, we
        // assume that a camera with that name was never connected in the first place.
        if (!heartbeatEntry.exists()) {
            Set<String> cameraNames = cameraTable.getInstance().getTable(kTableName).getSubTables();
            if (cameraNames.isEmpty()) {
                DriverStation.reportError(
                        "Could not find any PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!",
                        false);
            } else {
                DriverStation.reportError(
                        "PhotonVision coprocessor at path "
                                + path
                                + " not found on NetworkTables. Double check that your camera names match!",
                        true);
                DriverStation.reportError(
                        "Found the following PhotonVision cameras on NetworkTables:\n"
                                + String.join("\n", cameraNames),
                        false);
            }
        }
        // Check for connection status. Warn if disconnected.
        else if (!isConnected()) {
            DriverStation.reportWarning(
                    "PhotonVision coprocessor at path " + path + " is not sending new data.", true);
        }

        // Check for version. Warn if the versions aren't aligned.
        String versionString = versionEntry.get("");
        if (!versionString.isEmpty() && !PhotonVersion.versionMatches(versionString)) {
            // Error on a verified version mismatch
            // But stay silent otherwise

            String bfw = "\n\n\n\n\n"
                    + ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"
                    + ">>> !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                    + ">>>                                          \n"
                    + ">>> You are running an incompatible version  \n"
                    + ">>> of PhotonVision on your coprocessor!     \n"
                    + ">>>                                          \n"
                    + ">>> This is neither tested nor supported.    \n"
                    + ">>> You MUST update PhotonVision,            \n"
                    + ">>> PhotonLib, or both.                      \n"
                    + ">>>                                          \n"
                    + ">>> Your code will now crash.                \n"
                    + ">>> We hope your day gets better.            \n"
                    + ">>>                                          \n"
                    + ">>> !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
                    + ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";

            DriverStation.reportWarning(bfw, false);
            var versionMismatchMessage = "Photon version "
                    + PhotonVersion.versionString
                    + " does not match coprocessor version "
                    + versionString
                    + "!";
            DriverStation.reportError(versionMismatchMessage, false);
            throw new UnsupportedOperationException(versionMismatchMessage);
        }
    }
}
