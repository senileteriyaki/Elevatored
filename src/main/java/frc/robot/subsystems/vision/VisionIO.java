package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public boolean connected = false;
        public boolean hasTarget = false;
        public boolean megaTag1 = false;

        public double targetXDegrees = 0.0;
        public double targetYDegrees = 0.0;
        public double targetAreaPercent = 0.0;
        public int[] visibleTagIds = new int[0];

        public int targetId = 0;

        public PoseObservation[] poseObservations = new PoseObservation[0];

        public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity,
                int tagCount, double avgTagDistance, PoseObservationType type) {
        }

        public enum PoseObservationType {
            MEGATAG_1, MEGATAG_2, PHOTONVISION
        }
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void setPipeline(int index) {}

    default void setFiducialFilters(int[] ids) {}

    default void setRobotOrientation(Rotation2d heading) {} 

    default void updatePose(Pose2d robotPose) {}
}
