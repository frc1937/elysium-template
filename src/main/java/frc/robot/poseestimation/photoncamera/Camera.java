package frc.robot.poseestimation.photoncamera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

import static frc.robot.poseestimation.poseestimator.PoseEstimatorConstants.*;

public class Camera extends PhotonCameraIO {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public Camera(String cameraName, Transform3d robotCenterToCamera) {
        super(cameraName, robotCenterToCamera);

        photonCamera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
                APRIL_TAG_FIELD_LAYOUT,
                PRIMARY_POSE_STRATEGY,
                photonCamera,
                robotCenterToCamera
        );

        photonPoseEstimator.setTagModel(TAG_MODEL);
        photonPoseEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
    }

    private void logVisibleTags(boolean hasResult, Optional<EstimatedRobotPose> optionalEstimatedRobotPose) {
        if (!hasResult) {
            Logger.recordOutput("VisibleTags/" + photonCamera.getName(), new Pose2d[0]);
            return;
        }

        final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();
        final Pose2d[] visibleTagPoses = new Pose2d[estimatedRobotPose.targetsUsed.size()];

        for (int i = 0; i < visibleTagPoses.length; i++) {
            final int currentId = estimatedRobotPose.targetsUsed.get(i).getFiducialId();
            final Pose2d currentPose = TAG_ID_TO_POSE.get(currentId).toPose2d();

            visibleTagPoses[i] = currentPose;
        }

        Logger.recordOutput("VisibleTags/" + photonCamera.getName(), visibleTagPoses);
    }

    private boolean hasResult(Optional<EstimatedRobotPose> optionalEstimatedRobotPose) {
        if (optionalEstimatedRobotPose.isEmpty())
            return false;

        final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

        if (estimatedRobotPose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR) return true;

        return estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity() < MAXIMUM_AMBIGUITY;
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
    protected void refreshInputs(CameraInputsAutoLogged inputs) {
        if (photonCamera == null || !photonCamera.isConnected()) return;

        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();
        final Optional<EstimatedRobotPose> optionalEstimatedRobotPose = photonPoseEstimator.update(latestResult);

        inputs.hasResult = hasResult(optionalEstimatedRobotPose);

        if (inputs.hasResult && optionalEstimatedRobotPose.isPresent()) {
            final EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

            inputs.cameraPose = estimatedRobotPose.estimatedPose;
            inputs.lastResultTimestamp = estimatedRobotPose.timestampSeconds;
            inputs.visibleTags = estimatedRobotPose.targetsUsed.size();
            inputs.averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
        } else {
            inputs.visibleTags = 0;
            inputs.cameraPose = new Pose3d();
        }

        logVisibleTags(inputs.hasResult, optionalEstimatedRobotPose);
    }
}
