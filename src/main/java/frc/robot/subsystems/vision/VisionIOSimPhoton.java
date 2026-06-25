package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservationType;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSimPhoton implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final PhotonPoseEstimator poseEstimator;

    private final Transform3d robotToCamera;

    private Pose2d robotPose2d;

    private AprilTagFieldLayout layout;

    public VisionIOSimPhoton(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;

        camera = new PhotonCamera("limelight-mvrt");

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
        props.setFPS(90);
        props.setAvgLatencyMs(11);
        props.setLatencyStdDevMs(2);

        cameraSim = new PhotonCameraSim(camera, props);

        layout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        visionSim = new VisionSystemSim("vision");
        visionSim.addAprilTags(layout);
        visionSim.addCamera(cameraSim, robotToCamera);

        poseEstimator = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
        robotPose2d = new Pose2d();
    }

    /** MUST be called from simulationPeriodic */
    @Override
    public void updatePose(Pose2d robotPose) {
        visionSim.update(robotPose);
        robotPose2d = robotPose;

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result == null) {
            inputs.connected = false;
            return;
        }

        inputs.connected = true;

        if (!result.hasTargets()) {
            inputs.hasTarget = false;
            inputs.visibleTagIds = new int[0];
            inputs.poseObservations = new PoseObservation[0];
            return;
        }

        inputs.hasTarget = true;

        PhotonTrackedTarget best = result.getBestTarget();
        inputs.targetXDegrees = best.getYaw();
        inputs.targetYDegrees = best.getPitch();
        inputs.targetAreaPercent = best.getArea();
        inputs.targetId = best.getFiducialId();

        inputs.visibleTagIds = result.getTargets().stream()
                .mapToInt(PhotonTrackedTarget::getFiducialId)
                .toArray();

        List<PoseObservation> observations = new ArrayList<>();
        EstimatedRobotPose est = poseEstimator.update(result).orElse(null);

        if (est != null && !est.targetsUsed.isEmpty()) {
            double avgDist = est.targetsUsed.stream()
                    .mapToDouble(t ->
                            t.getBestCameraToTarget()
                             .getTranslation()
                             .getNorm())
                    .average()
                    .orElse(Double.POSITIVE_INFINITY);

            boolean megaTag1 = avgDist < 3.6;

            observations.add(new PoseObservation(
                    est.timestampSeconds,
                    est.estimatedPose,
                    0.0,
                    est.targetsUsed.size(),
                    avgDist,
                    megaTag1
                            ? PoseObservationType.MEGATAG_1
                            : PoseObservationType.MEGATAG_2));
        }

        inputs.poseObservations =
                observations.toArray(PoseObservation[]::new);
        
        List<Pose3d> sightLines = new ArrayList<>();

        Pose3d cameraPose = new Pose3d(robotPose2d).plus(robotToCamera);

        for (PhotonTrackedTarget target : result.getTargets()) {
            int id = target.getFiducialId();
            var tagPoseOpt = layout.getTagPose(id);
            if (tagPoseOpt.isEmpty()) continue;

            sightLines.add(cameraPose);
            sightLines.add(tagPoseOpt.get());
        }

        Logger.recordOutput("Vision/Photon/SightLines",
                sightLines.toArray(new Pose3d[0]));

    }

    @Override
    public void setPipeline(int index) {}

    @Override
    public void setFiducialFilters(int[] ids) {}

    @Override
    public void setRobotOrientation(Rotation2d heading) {}

    
}