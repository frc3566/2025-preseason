package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.vision.SupplyAprilTagPose;


/* Make sure that 
 * • the correct camera resolution is selected 
 * • the targets are sorted by closest to farthest distance
 * on https://photonvision.local:5800 
*/

public class Vision extends SubsystemBase {
    private PhotonCamera apriltagCamera;
    private PhotonPoseEstimator poseEstimator;
    private Pose2d targetPose;

    private static final List<Integer> BLUE_APRILTAG_IDS = List.of(7, 6, 1, 2, 14, 15, 16);
    private static final List<Integer> RED_APRILTAG_IDS = List.of(4, 5, 9, 10, 11, 12, 13);
    
    /* list of fiducial ids to look for depending on alliance */
    private List<Integer> targetFiducialIds = List.of(4);

    public Vision() {
        apriltagCamera = new PhotonCamera(Constants.Vision.aprilTagCameraName);
        poseEstimator = new PhotonPoseEstimator(
            Constants.Vision.aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            apriltagCamera,
            Constants.Vision.robotToCamera
        );

        refreshTargetFiducialIds();
        System.out.println("Targetting fiducial ids: " + targetFiducialIds);
    }

    public List<Integer> refreshTargetFiducialIds() {
        DriverStation.getAlliance().map(alliance -> {
            if (alliance == DriverStation.Alliance.Blue) { targetFiducialIds = BLUE_APRILTAG_IDS; }
            else if (alliance == DriverStation.Alliance.Red) { targetFiducialIds = RED_APRILTAG_IDS; }
            return null;
        });

        return targetFiducialIds;
    }

    /**
     * Gets the closest April Tag that matches any id in the list of targetFiducialIds
     *  if and only if its ambiguity < 0.2
     * 
     * @return Optional<PhotonTrackedTarget>: The closest April Tag that matches a target fiducial id if its ambiguity < 0.2
     */
    public Optional<PhotonTrackedTarget> getAprilTag() {
        refreshTargetFiducialIds();
        
        var result = apriltagCamera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        Optional<PhotonTrackedTarget> target = Optional.empty();

        /* get closest target that matches any targetFiducialId */
        for (PhotonTrackedTarget potentialTarget: targets) {
            if (targetFiducialIds.contains(potentialTarget.getFiducialId())) {
                target = Optional.of(potentialTarget);
                break;
            }
        }

        /* return target only if ambiguity < 0.2 */
        return target.map(e -> 
            e.getPoseAmbiguity() < 0.2 ? e : null
        );
    }

    public void writePose(Pose2d pose) {
        targetPose = pose;
    }

    public static Rotation2d limitRange(Rotation2d angle, double minDegrees, double maxDegrees) {
        return Rotation2d.fromDegrees(Math.min(Math.max(angle.getDegrees(), minDegrees), maxDegrees));
    }
    
    public Pose2d getPose() {
        return targetPose;
    }

    public void resetPose() {
        targetPose = new Pose2d();
        
    }

    public Command defaultCommand() {
        return new SupplyAprilTagPose(this, new Pose2d(), this::writePose);
    }

    public Pose2d getPoseTo(PhotonTrackedTarget target) {
        Transform3d transform = target.getBestCameraToTarget();
        Translation2d end = transform.getTranslation().toTranslation2d();

        double zAngleTheta = transform.getRotation().getZ();
        Rotation2d yaw = Rotation2d.fromRadians(Math.signum(zAngleTheta) * (Math.PI - Math.abs(zAngleTheta))).unaryMinus();

        return new Pose2d(end, yaw);
    }

    public Optional<Transform3d> getMultiAprilTag() {
        var result = apriltagCamera.getLatestResult().getMultiTagResult();
        if (!result.estimatedPose.isPresent) { return Optional.empty(); }

        Transform3d fieldToCamera = result.estimatedPose.best;
        return Optional.of(fieldToCamera);
    }

    public Optional<Pose3d> estimatePose() {
        return poseEstimator.update().map(e -> e.estimatedPose);
    }

    public void printAllResults() {
        System.out.println("Vision log:");
        var result = apriltagCamera.getLatestResult();

        if (!result.hasTargets()) {
            System.out.println("> No targets found.");
            return;
        }

        System.out.println("> Single AprilTag: " + getAprilTag());
        System.out.println("> Multi AprilTag: " + getMultiAprilTag());
        System.out.println("> Estimated pose: " + estimatePose());
    }
}