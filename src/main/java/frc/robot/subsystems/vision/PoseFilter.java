package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.Util;

import java.util.ArrayList;
import java.util.List;

/**
 * Usage: 
 * // Init
 * PoseFilter pf = new PoseFilter();
 *
 * // Periodic
 * pf.pushMeasurement(pose0, uncertainty0);
 * pf.pushMeasurement(pose1, uncertainty1);
 * pf.pushMeasurement(pose2, uncertainty2);
 * 
 * UncertainPose up = pf.filter();
 * Pose2d robotPose = up.pose();
 * Uncertainty robotPose = up.uncertainty();
 */
public class PoseFilter {

    // An uncertainty of {1.0, 10.0} means we believe the pose is accurate to 1 meter and 10 degrees
    // An uncertainty of INFINITE means ignore this pose for this measurement
    // An uncertainty of CERTAIN means this value is 100% accurate and put all trust here
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
    
    // TODO: implement ring buffer for history 
    private List<UncertainPose> currentFrameMeasurements;

    public PoseFilter() {
        currentFrameMeasurements = new ArrayList<UncertainPose>(10);
    }

    public void pushMeasurement(Pose2d pose, Uncertainty uncertainty) {
        currentFrameMeasurements.add(new UncertainPose(pose, uncertainty));
    }

    public UncertainPose filter() {
        double x = 0;
        double y = 0;
        double deg = 0;

        double minTransUnc = Uncertainty.INFINITE;
        double minRotUnc = Uncertainty.INFINITE;
        int minTransIndex = -1;
        int minRotIndex = -1;

        for (int i = 0; i < currentFrameMeasurements.size(); i++) {
            Uncertainty unc = currentFrameMeasurements.get(i).uncertainty();
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

        if (minTransUnc == Uncertainty.CERTAIN) {
            x = currentFrameMeasurements.get(minTransIndex).pose().getX();
            y = currentFrameMeasurements.get(minTransIndex).pose().getY();
        } else {
            double weightedX = 0.0;
            double weightedY = 0.0;
            double uncertaintySum = 0.0;

            for (int i = 0; i < currentFrameMeasurements.size(); i++) {
                Uncertainty unc = currentFrameMeasurements.get(i).uncertainty();
                double translationalUncertainty = unc.translationalMeters();
                double currX = currentFrameMeasurements.get(i).pose().getX();
                double currY = currentFrameMeasurements.get(i).pose().getY();

                if (translationalUncertainty != Uncertainty.INFINITE) {
                    double scale = 1.0 / (translationalUncertainty * translationalUncertainty);
                    weightedX += scale * currX;
                    weightedY += scale * currY;
                    uncertaintySum += scale;
                }
            }

            if (uncertaintySum > 0) {
                x = weightedX / uncertaintySum;
                y = weightedY / uncertaintySum;
            }
        }

        if (minRotUnc == Uncertainty.CERTAIN) {
            deg = currentFrameMeasurements.get(minRotIndex).pose().getRotation().getDegrees();
        } else {
            double weightedDeg = 0.0;
            double uncertaintySum = 0.0;

            for (int i = 0; i < currentFrameMeasurements.size(); i++) {
                Uncertainty unc = currentFrameMeasurements.get(i).uncertainty();
                double rotationalUncertainty = unc.rotationalDegrees();
                double currDeg = currentFrameMeasurements.get(i).pose().getRotation().getDegrees();

                if (rotationalUncertainty != Uncertainty.INFINITE) {
                    double scale = 1.0 / (rotationalUncertainty * rotationalUncertainty);
                    weightedDeg += scale * currDeg;
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