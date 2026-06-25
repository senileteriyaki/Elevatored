package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Util;

import java.util.ArrayList;
import java.util.List;

public class PoseFilterV2 {

    // ========== Filtering Constants ==========
    private static final double MAX_AMBIGUITY = 0.3;
    private static final double MAX_Z_ERROR = 0.75;
    private static final double LINEAR_STD_DEV_BASELINE = 0.02;
    private static final double ANGULAR_STD_DEV_BASELINE = 0.06;
    private static final double LINEAR_STD_DEV_MEGATAG2_FACTOR = 0.5;
    private static final double MAX_DISTANCE_PER_TAG = 4.0;

    // ========== Gyro Validation Constants ==========
    private static final double GYRO_POSITION_THRESHOLD_M = 0.5;
    private static final double GYRO_ROTATION_THRESHOLD_DEG = 30.0;

    // ========== MT1/MT2 Selection Constants ==========
    private static final int MT2_MIN_TAG_COUNT = 2;

    // Per-camera state
    private Pose2d lastMT1Pose = new Pose2d();
    private Pose2d lastMT2Pose = new Pose2d();
    private boolean gyroMisconfigWarning = false;

    public static class Uncertainty {
        public final static double INFINITE = 75588557.0;
        public final static double CERTAIN = 0.0;

        public final double translational_meters;
        public final double rotational_degrees;

        public Uncertainty(double translational_meters, double rotational_degrees) {
            this.translational_meters = Util.limit(translational_meters, CERTAIN, INFINITE);
            this.rotational_degrees = Util.limit(rotational_degrees, CERTAIN, INFINITE);
        }

        public double translationalMeters() {
            return translational_meters;
        }

        public double rotationalDegrees() {
            return rotational_degrees;
        }
    }

    public record UncertainPose(Pose2d pose, Uncertainty uncertainty) {}

    public record VisionInput(
        Pose2d pose,
        int tagCount,
        double avgTagDistance,
        double ambiguity,
        PoseObservationType type
    ) {}

    public enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2
    }

    private List<VisionInput> currentFrameMeasurements;

    public PoseFilterV2() {
        currentFrameMeasurements = new ArrayList<>(10);
    }

    /**
     * Calculate standard deviations based on measurement quality.
     * Uses: stdDev = baseline * (distance^2 / tagCount)
     */
    public static Uncertainty calculateStdDev(Pose2d pose, int tagCount, double avgTagDistance, PoseObservationType type) {
        double stdDevFactor = Math.pow(avgTagDistance, 2.0) / Math.max(tagCount, 1);
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

        // MT2 is more accurate than MT1
        if (type == PoseObservationType.MEGATAG_2) {
            linearStdDev *= LINEAR_STD_DEV_MEGATAG2_FACTOR;
        }

        return new Uncertainty(linearStdDev, angularStdDev);
    }

    /**
     * Check if a measurement passes all rejection filters
     */
    public static boolean shouldRejectMeasurement(Pose2d pose, int tagCount, double avgTagDistance, double ambiguity) {
        // No tags
        if (tagCount == 0) return true;

        // Single tag with high ambiguity
        if (tagCount == 1 && ambiguity > MAX_AMBIGUITY) return true;

        // Out of field bounds
        if (pose.getX() < 0 || pose.getX() > 16.54 || pose.getY() < 0 || pose.getY() > 8.02) return true;

        // Distance per tag threshold
        if (avgTagDistance > MAX_DISTANCE_PER_TAG * tagCount) return true;

        return false;
    }

    /**
     * Update gyro validation state with new MT1 and MT2 poses
     */
    public void updateGyroValidation(Pose2d mt1Pose, Pose2d mt2Pose) {
        this.lastMT1Pose = mt1Pose;
        this.lastMT2Pose = mt2Pose;

        double positionDelta = mt1Pose.getTranslation().getDistance(mt2Pose.getTranslation());
        
        double rotationDelta = Math.abs(
            mt1Pose.getRotation().minus(mt2Pose.getRotation()).getDegrees()
        );
        if (rotationDelta > 180) rotationDelta = 360 - rotationDelta;

        gyroMisconfigWarning = positionDelta > GYRO_POSITION_THRESHOLD_M 
                            || rotationDelta > GYRO_ROTATION_THRESHOLD_DEG;
    }

    /**
     * Returns true if gyro validation detected a problem
     */
    public boolean isGyroMisconfigured() {
        return gyroMisconfigWarning;
    }

    public void pushMeasurement(VisionInput input) {
        currentFrameMeasurements.add(input);
    }

    public UncertainPose filter() {
        double x = 0;
        double y = 0;
        double deg = 0;

        // Separate MT1 and MT2 measurements
        List<VisionInput> mt1Measurements = new ArrayList<>();
        List<VisionInput> mt2Measurements = new ArrayList<>();

        for (VisionInput input : currentFrameMeasurements) {
            if (input.type() == PoseObservationType.MEGATAG_1) {
                mt1Measurements.add(input);
            } else {
                mt2Measurements.add(input);
            }
        }

        // Update gyro validation
        if (!mt1Measurements.isEmpty() && !mt2Measurements.isEmpty()) {
            updateGyroValidation(
                mt1Measurements.get(mt1Measurements.size() - 1).pose(),
                mt2Measurements.get(mt2Measurements.size() - 1).pose()
            );
        }

        // Smart MT1/MT2 selection
        List<VisionInput> selectedMeasurements;
        if (gyroMisconfigWarning) {
            // Fall back to MT1 if gyro is misconfigured
            selectedMeasurements = mt1Measurements;
        } else if (currentFrameMeasurements.stream().anyMatch(m -> m.tagCount() >= MT2_MIN_TAG_COUNT)) {
            // Prefer MT2 when we have enough tags
            selectedMeasurements = mt2Measurements.isEmpty() ? mt1Measurements : mt2Measurements;
        } else {
            selectedMeasurements = currentFrameMeasurements;
        }

        // Find minimum uncertainties
        double minTransUnc = Uncertainty.INFINITE;
        double minRotUnc = Uncertainty.INFINITE;
        int minTransIndex = -1;
        int minRotIndex = -1;

        for (int i = 0; i < selectedMeasurements.size(); i++) {
            VisionInput input = selectedMeasurements.get(i);
            Uncertainty unc = calculateStdDev(input.pose(), input.tagCount(), input.avgTagDistance(), input.type());
            double currTran = unc.translationalMeters();
            double currRot = unc.rotationalDegrees();

            if (currTran < minTransUnc) {
                minTransUnc = currTran;
                minTransIndex = i;
            }
            if (currRot < minRotUnc) {
                minRotUnc = currRot;
                minRotIndex = i;
            }
        }

        Uncertainty robotUncertainty = new Uncertainty(minTransUnc, minRotUnc);

        // Translational filtering
        if (minTransUnc == Uncertainty.CERTAIN || selectedMeasurements.isEmpty()) {
            if (minTransIndex >= 0) {
                x = selectedMeasurements.get(minTransIndex).pose().getX();
                y = selectedMeasurements.get(minTransIndex).pose().getY();
            }
        } else {
            double weightedX = 0.0;
            double weightedY = 0.0;
            double uncertaintySum = 0.0;

            for (int i = 0; i < selectedMeasurements.size(); i++) {
                VisionInput input = selectedMeasurements.get(i);
                Uncertainty unc = calculateStdDev(input.pose(), input.tagCount(), input.avgTagDistance(), input.type());
                double translationalUncertainty = unc.translationalMeters();

                if (translationalUncertainty != Uncertainty.INFINITE) {
                    double scale = 1.0 / (translationalUncertainty * translationalUncertainty);
                    weightedX += scale * input.pose().getX();
                    weightedY += scale * input.pose().getY();
                    uncertaintySum += scale;
                }
            }

            if (uncertaintySum > 0) {
                x = weightedX / uncertaintySum;
                y = weightedY / uncertaintySum;
            }
        }

        // Rotational filtering
        if (minRotUnc == Uncertainty.CERTAIN || selectedMeasurements.isEmpty()) {
            if (minRotIndex >= 0) {
                deg = selectedMeasurements.get(minRotIndex).pose().getRotation().getDegrees();
            }
        } else {
            double weightedDeg = 0.0;
            double uncertaintySum = 0.0;

            for (int i = 0; i < selectedMeasurements.size(); i++) {
                VisionInput input = selectedMeasurements.get(i);
                Uncertainty unc = calculateStdDev(input.pose(), input.tagCount(), input.avgTagDistance(), input.type());
                double rotationalUncertainty = unc.rotationalDegrees();

                if (rotationalUncertainty != Uncertainty.INFINITE) {
                    double scale = 1.0 / (rotationalUncertainty * rotationalUncertainty);
                    weightedDeg += scale * input.pose().getRotation().getDegrees();
                    uncertaintySum += scale;
                }
            }

            if (uncertaintySum > 0) {
                deg = weightedDeg / uncertaintySum;
            }
        }

        currentFrameMeasurements.clear();
        return new UncertainPose(new Pose2d(x, y, Rotation2d.fromDegrees(deg)), robotUncertainty);
    }
}
