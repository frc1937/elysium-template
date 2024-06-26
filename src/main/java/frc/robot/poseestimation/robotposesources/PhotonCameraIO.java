package frc.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.poseestimation.photonposeestimator.EstimatedRobotPose;
import frc.robot.poseestimation.photonposeestimator.PhotonPoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.poseestimation.PoseEstimatorConstants.*;

public class PhotonCameraIO extends RobotPoseSourceIO {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    protected PhotonCameraIO(String cameraName, Transform3d robotCenterToCamera) {
        photonCamera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                APRIL_TAG_FIELD_LAYOUT,
                PRIMARY_POSE_STRATEGY,
                photonCamera,
                robotCenterToCamera
        );

        photonPoseEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
        photonPoseEstimator.setTagModel(TAG_MODEL);
    }

    private void logVisibleTags(boolean hasResult, EstimatedRobotPose estimatedRobotPose) {
        if (!hasResult || estimatedRobotPose == null) {
            Logger.recordOutput("VisibleTags/" + photonCamera.getName());
            return;
        }

        final Pose2d[] visibleTagPoses = new Pose2d[estimatedRobotPose.targetsUsed().size()];

        for (int i = 0; i < visibleTagPoses.length; i++) {
            final int currentId = estimatedRobotPose.targetsUsed().get(i).getFiducialId();
            final Pose2d currentPose = TAG_ID_TO_POSE.get(currentId).toPose2d();

            visibleTagPoses[i] = currentPose;
        }

        Logger.recordOutput("VisibleTags/" + photonCamera.getName(), visibleTagPoses);
    }

    private boolean hasResult(EstimatedRobotPose estimatedRobotPose) {
        if (estimatedRobotPose == null) return false;

        if (estimatedRobotPose.strategy() == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            return true;

        return estimatedRobotPose.targetsUsed().get(0).getPoseAmbiguity() < MAXIMUM_AMBIGUITY;
    }

    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        double distanceSum = 0;

        for (PhotonTrackedTarget currentTarget : targets) {
            final Translation2d distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            distanceSum += distanceTranslation.getNorm();
        }

        return distanceSum / targets.size();
    }

    @Override
    protected void refreshInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();
        Optional<EstimatedRobotPose> optionalEstimatedRobotPose = photonPoseEstimator.update(latestResult);

        inputs.hasResult = hasResult(optionalEstimatedRobotPose.orElse(null));

        if (inputs.hasResult) {
            final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

            inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(estimatedRobotPose.estimatedPose());
            inputs.lastResultTimestamp = estimatedRobotPose.timestampSeconds();
            inputs.visibleTags = estimatedRobotPose.targetsUsed().size();
            inputs.averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
        } else {
            inputs.visibleTags = 0;
            inputs.cameraPose = new double[0];
        }

        logVisibleTags(inputs.hasResult, optionalEstimatedRobotPose.orElse(null));
    }
}
