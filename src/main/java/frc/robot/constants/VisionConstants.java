package frc.robot.constants;

import java.util.Set;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class VisionConstants {
    public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)); // TODO: Update these values to match where the camera is

    public static final Set<Integer> RED_HUB_TAGS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    public static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    public static final double BASE_STD_DEV = 0.1;
    public static final double DISTANCE_WEIGHT = 0.1;
    public static final double AMBIGUITY_WEIGHT = 10.0;
}
