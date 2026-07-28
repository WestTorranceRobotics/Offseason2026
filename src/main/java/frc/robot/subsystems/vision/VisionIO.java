package frc.robot.subsystems.vision;

import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

public interface VisionIO {
    public static class VisionIOInputs {
        public List<PhotonPipelineResult> results;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    public void updateInputs(VisionIOInputs inputs);
}
