package frc.robot.subsystems.vision;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservationType;

public class VisionIOLimelight implements VisionIO {
    private final String cameraName;
    private PoseObservation mt1;
    private PoseObservation mt2;
    private Optional<Alliance> alliance;
    private boolean red;

    public VisionIOLimelight() {
        this.cameraName = "limelight-sean";
        mt1 = null;
        mt2 = null;

        alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                red = true;
            } else {
                red = false;
            }
        } else {
            red = false;
        }

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(cameraName);
        inputs.connected = results != null;

        if (!inputs.connected) {
            return;
        }

        inputs.hasTarget = LimelightHelpers.getTV(cameraName);

        LimelightHelpers.PoseEstimate estimateMT1 =
                LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

        if (estimateMT1 != null) {
            double distMeters = estimateMT1.avgTagDist;
            if (distMeters < 0.2) {
                inputs.megaTag1 = true;
            } else {
                inputs.megaTag1 = false;
            }
        }

        if (inputs.megaTag1) {
            mt1 = null;
            if (estimateMT1 != null && estimateMT1.tagCount > 0) {
                mt1 = new PoseObservation(estimateMT1.timestampSeconds,
                        new Pose3d(estimateMT1.pose), 0.0,
                        estimateMT1.tagCount, estimateMT1.avgTagDist,
                        PoseObservationType.MEGATAG_1);
            }
            Logger.recordOutput("Vision/PoseEstimate", estimateMT1.pose);
        } else {
            mt2 = null;
            LimelightHelpers.PoseEstimate estimateMT2 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
            if (estimateMT2 != null && estimateMT2.tagCount > 0) {
                mt2 = new PoseObservation(estimateMT2.timestampSeconds,
                        new Pose3d(estimateMT2.pose), 0.0, estimateMT2.tagCount,
                        estimateMT2.avgTagDist, PoseObservationType.MEGATAG_2);
                Logger.recordOutput("Vision/PoseEstimate", estimateMT2.pose);
            }
        }

        if (mt1 != null && mt2 != null) {
            inputs.poseObservations = new PoseObservation[] {mt1, mt2};
        } else if (mt1 != null) {
            inputs.poseObservations = new PoseObservation[] {mt1};
        } else if (mt2 != null) {
            inputs.poseObservations = new PoseObservation[] {mt2};
        } else {
            inputs.poseObservations = new PoseObservation[0];
        }

    }

    @Override
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(cameraName, index);
    }

    @Override
    public void setFiducialFilters(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, ids);
    }

    @Override
    public void setRobotOrientation(Rotation2d heading) {
        LimelightHelpers.SetRobotOrientation(cameraName, heading.getDegrees(), 0, 0, 0, 0, 0);
    }

    @Override
    public void updatePose(Pose2d robotPose) {
    }
}
