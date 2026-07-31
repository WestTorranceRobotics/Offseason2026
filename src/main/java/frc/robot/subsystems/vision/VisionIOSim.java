package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;
import static org.wpilib.units.Units.*;

import frc.robot.RobotContainer;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;

public class VisionIOSim implements VisionIO {
    private final VisionSystemSim visionSystemSim = new VisionSystemSim("main");
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;

    public VisionIOSim() {
        SimCameraProperties cameraProps = new SimCameraProperties();

        // Diagonal FOV was calculated from horizontal and vertical FOV given from
        // https://www.arducam.com/ov9281-mipi-1mp-monochrome-global-shutter-camera-module-m12-mount-lens-raspberry-pi
        cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
        cameraProps.setCalibError(0.25, 0.08);
        cameraProps.setFPS(15);
        cameraProps.setAvgLatencyMs(16);
        cameraProps.setLatencyStdDevMs(5);

        camera = new PhotonCamera(CAMERA_NAME);
        cameraSim = new PhotonCameraSim(camera, cameraProps);

        Rotation3d cameraRotation =
                new Rotation3d(0, Degrees.of(-15).in(Radians), Degrees.of(180).in(Radians));
        Transform3d cameraPosition = new Transform3d(new Translation3d(-0.3, 0, 0.5), cameraRotation);

        visionSystemSim.addCamera(cameraSim, cameraPosition);

        cameraSim.enableProcessedStream(true);
        cameraSim.enableRawStream(true);
        cameraSim.setTargetSortMode(PhotonTargetSortMode.Centermost);

        cameraSim.enableDrawWireframe(true);

        try {
            visionSystemSim.addAprilTags(
                    AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.resourceFile));
        } catch (IOException ioe) {
            System.out.println("Failed to load april tag field");
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        updateSim();
        inputs.results = camera.getAllUnreadResults();
    }

    public void updateSim() {
        visionSystemSim.update(RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose());
    }
}
