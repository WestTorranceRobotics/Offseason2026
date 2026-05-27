package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;
import static org.ironmaple.utils.FieldMirroringUtils.isSidePresentedAsRed;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.EstimateConsumer;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();
    private final EstimateConsumer estConsumer;

    List<PhotonTrackedTarget> trackedTargets;
    PhotonTrackedTarget bestTarget;
    PhotonPoseEstimator photonEstimator =
            new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), ROBOT_TO_CAM);
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

    public Vision(VisionIO io, EstimateConsumer estConsumer) {
        this.io = io;
        this.estConsumer = estConsumer;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (!inputs.results.isEmpty()) {
            PhotonPipelineResult result = inputs.results.get(inputs.results.size() - 1);
            if (result.hasTargets()) {
                trackedTargets = result.getTargets();
                bestTarget = result.getBestTarget();

                estimatedPose = photonEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isEmpty()) estimatedPose = photonEstimator.estimateClosestToCameraHeightPose(result);
            }
        }
        fuseVisionPoseEstimates();
    }

    private PhotonTrackedTarget getTrackedTarget(int targetID) {
        if (trackedTargets != null && !trackedTargets.isEmpty()) {
            for (PhotonTrackedTarget trackedTarget : trackedTargets) {
                if (trackedTarget.fiducialId == targetID) {
                    return trackedTarget;
                }
            }
        }
        return null;
    }

    public Optional<Pose2d> getTargetPoseOfAprilTag(int targetID) {
        Pose2d pose = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded)
                .getTagPose(targetID)
                .get()
                .toPose2d();

        if (pose != null) return Optional.of(pose);
        return Optional.empty();
    }

    public Optional<Double> getDistance(int targetID) {
        if (getTrackedTarget(targetID) == null) return Optional.empty();
        return Optional.of(getTrackedTarget(targetID)
                .getBestCameraToTarget()
                .getTranslation()
                .getNorm());
    }

    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    public Optional<Double> getYaw(int targetID) {
        PhotonTrackedTarget target = getTrackedTarget(targetID);
        if (target != null) return Optional.of(target.getYaw());
        return Optional.empty();
    }

    public Optional<Double> getPitch(int targetID) {
        PhotonTrackedTarget target = getTrackedTarget(targetID);
        if (target != null) return Optional.of(target.getPitch());
        return Optional.empty();
    }

    public int getHubAprilTagID() {
        return isSidePresentedAsRed() ? 10 : 25;
    }

    public double getYawOfHub() {
        return getYaw(getHubAprilTagID()).orElse(0.0);
    }

    public void fuseVisionPoseEstimates() {
        estimatedPose.ifPresent(est -> {
            int tagCount = est.targetsUsed.size();
            double averageDistance = 0;
            double maxAmbiguity = est.targetsUsed.stream()
                    .mapToDouble(target -> target.getPoseAmbiguity())
                    .max()
                    .orElse(1.0);
            for (PhotonTrackedTarget target : est.targetsUsed) {
                averageDistance +=
                        target.getBestCameraToTarget().getTranslation().getNorm();
            }

            averageDistance /= tagCount;

            double xyStdDev = BASE_STD_DEV
                    * (1.0 + (averageDistance * DISTANCE_WEIGHT))
                    * (1.0 + (maxAmbiguity * AMBIGUITY_WEIGHT))
                    / Math.sqrt(tagCount);

            estConsumer.accept(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE));
        });
    }
}
