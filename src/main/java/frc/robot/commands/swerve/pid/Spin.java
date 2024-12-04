package frc.robot.commands.swerve.pid;


import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.WithStatus;
import frc.robot.subsystems.Swerve;

/**
 * Custom command to rotate swerve to a target rotation, utilizing drive() from the Swerve subsystem. 
 * Calculates the velocity of rotation using PID from the supplied final pose.
 */
public class Spin extends Command implements WithStatus {
    private Swerve s_Swerve;
    private Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;

    private ProfiledPIDController thetaController;

    private boolean isRunning;

    private static class SpinCommandConstants {
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.5 * Math.PI; 
        public static final double kMaxAngularSpeedRadiansPerSecondSquared =  2.25 * Math.PI; 
        public static final double kPThetaController = 6;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
        );
    }

    public Spin(Swerve s_Swerve, Supplier<Pose2d> targetPoseSupplier) {
        this.s_Swerve = s_Swerve;
        this.targetPoseSupplier = targetPoseSupplier;

        isRunning = false;

        addRequirements(s_Swerve);   
    }

    @Override
    public void initialize() {
        isRunning = true;
        targetPose = targetPoseSupplier.get();

        this.thetaController = new ProfiledPIDController(
            SpinCommandConstants.kPThetaController, 0, 0, SpinCommandConstants.kThetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        s_Swerve.zeroGyro();
        s_Swerve.resetOdometry(new Pose2d());
        thetaController.reset(new Pose2d().getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_Swerve.getPose();
        System.out.println("Spin: " + currentPose);

        double thetaVelocity = thetaController.atGoal() ? 0.0 : 
            thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        s_Swerve.drive(new Translation2d(), thetaVelocity, true, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        s_Swerve.drive(new Translation2d(), 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }

    public boolean atGoal() {
        return isRunning && thetaController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}