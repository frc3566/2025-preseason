package frc.robot.commands.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.WithStatus;
import frc.robot.subsystems.Vision;

public class SupplyAprilTagPose extends Command implements WithStatus {
    private Vision s_Vision;
    
    private Consumer<Pose2d> setTargetPose;

    private int counter = 0;
    private boolean targetPoseSet = false;
    private boolean isRunning = false;

    public static final int MAX_CYCLE_COUNT = 10;

    private static final double
        cameraToRobotFront = 0.5,
        speakerAprilTagGap = 1,
        additionalGapForGoodMeasure = 0.775;

    public SupplyAprilTagPose(Vision s_Vision, Pose2d currentPose, Consumer<Pose2d> setTargetPose) {
        this.s_Vision = s_Vision;
        this.setTargetPose = setTargetPose;
        addRequirements(s_Vision);
    }

    @Override
    public void initialize() {
        targetPoseSet = false;
        isRunning = true;
        counter = 0;
    }

    /* TODO: take currentPose into account */
    @Override
    public void execute() {
        if (targetPoseSet) { return; }

        if (counter > MAX_CYCLE_COUNT) { this.cancel(); }
        
        var result = s_Vision.getAprilTag();
        
        if (result.isEmpty()) {
            counter += 1;
            System.out.println("Cycle: " + counter);
            return;
        }

        s_Vision.printAllResults();

        Pose2d poseToAprilTag = s_Vision.getPoseTo(result.get());
        System.out.println("> April Tag: " + poseToAprilTag);

        Pose2d poseToAprilTagMinusGap = new Pose2d(
            poseToAprilTag.getTranslation().minus(new Translation2d(
                cameraToRobotFront + speakerAprilTagGap + additionalGapForGoodMeasure, poseToAprilTag.getRotation())),
            poseToAprilTag.getRotation()
        );
        System.out.println("> April Tag minus gap: " + poseToAprilTagMinusGap);

        Pose2d singleDimensionTranslation = new Pose2d(
            poseToAprilTagMinusGap.getTranslation().rotateBy(poseToAprilTag.getRotation().unaryMinus()),
            new Rotation2d()
        );
        System.out.println("> Translation component: " + singleDimensionTranslation);

        setTargetPose.accept(poseToAprilTagMinusGap);
        targetPoseSet = true;
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
    }

    @Override
    public boolean isFinished() {
        return targetPoseSet;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }
}
