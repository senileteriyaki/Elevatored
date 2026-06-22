package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservation;

/**
 * Enhanced Vision subsystem for vision2 folder. Manages camera IO and provides automatic odometry
 * updates using MegaTag1 and MegaTag2.
 */
public class Vision {

    private final VisionIO io;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    public static final Transform3d ROBOT_TO_CAMERA =
            // new Transform3d(new Translation3d(0.16, -0.162, 0.45), new Rotation3d(0, Units.degreesToRadians(20), 0));

    new Transform3d(new Translation3d(0.0889, -0.2794, 0.4445), new Rotation3d(0, Units.degreesToRadians(20), 0));
    public static Vision getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Vision(new VisionIOLimelight());
                    break;
                case SIM:
                    instance = new Vision(new VisionIOSimPhoton(ROBOT_TO_CAMERA));
                    break;
                case REPLAY:
                    break;

                default:
                    break;
            }
        }
        return instance;
    }

    public Vision(VisionIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/", inputs);
    }

    /**
     * Check if vision has a new pose measurement this cycle
     */
    public boolean hasNewPose() {
        return inputs.poseObservations != null && inputs.poseObservations.length > 0;
    }

    /**
     * Get the latest vision pose (most recent measurement)
     */
    public edu.wpi.first.math.geometry.Pose2d getLatestPose() {
        if (hasNewPose()) {
            return inputs.poseObservations[0].pose().toPose2d();
        }
        return null;
    }

    /**
     * Get confidence score based on tag count and distance (0-1 scale) Higher = more confident
     */
    public double getConfidence() {
        if (!hasNewPose()) {
            return 0.0;
        }

        PoseObservation observation = inputs.poseObservations[0];

        // More tags = more confidence
        double tagCountConfidence = Math.min(observation.tagCount() / 4.0, 1.0);

        // Closer distance = more confidence
        double maxDistance = 8.0;
        double distanceConfidence =
                Math.max(1.0 - (observation.avgTagDistance() / maxDistance), 0.0);

        // Average the two factors
        return (tagCountConfidence + distanceConfidence) / 2.0;
    }

    public void updateOdometry(Drive drive) {
        io.updatePose(drive.getPose());
        io.setRobotOrientation(drive.getRotation());
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    public int[] getVisibleTagIds() {
        return inputs.visibleTagIds;
    }

    public PoseObservation[] getPoseObservations() {
        return inputs.poseObservations;
    }

    /** Horizontal angle to target in degrees (negative = left) */
    public double getTargetX() {
        return inputs.targetXDegrees;
    }

    /** Vertical angle to target in degrees (negative = down) */
    public double getTargetY() {
        return inputs.targetYDegrees;
    }

    /** Target area as percentage of image (0-100) */
    public double getTargetArea() {
        return inputs.targetAreaPercent;
    }

    /**
     * Convenience method to get the latest pose as a Pose2d.
     */
    public edu.wpi.first.math.geometry.Pose2d getLatestPose2d() {
        if (inputs.poseObservations.length > 0) {
            return inputs.poseObservations[0].pose().toPose2d();
        }
        return null;
    }

    public void setPipeline(int index) {
        io.setPipeline(index);
    }
}
