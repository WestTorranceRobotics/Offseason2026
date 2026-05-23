package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import org.photonvision.PhotonCamera;

public class VisionIOReal implements VisionIO {
    private final PhotonCamera camera;

    public VisionIOReal() {
        camera = new PhotonCamera(CAMERA_NAME);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.results = camera.getAllUnreadResults();
    }
}
