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
 Custom command to drive swerve to a target rotation, utilizing drive() from the Swerve subsystem. 
 * Calculates the velocity of the swerve using PID from the supplied final pose.
 * Utilizes Translation2d with the scalar value of movement and angle of drive instead of X and Y. 
 */
public class Drive extends Command implements WithStatus {
    private Swerve s_Swerve;
    private Supplier<Pose2d> targetPoseSupplier;
    private Pose2d targetPose;

    private ProfiledPIDController driveController;

    private boolean isRunning;

    private static class DriveCommandConstants {
        public static final double kPXController = 4; //8
        public static final double kMaxSpeedMetersPerSecond = 4; //5
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    }

    public Drive(Swerve s_Swerve, Supplier<Pose2d> targetPoseSupplier) {
        this.s_Swerve = s_Swerve;
        this.targetPoseSupplier = targetPoseSupplier;

        isRunning = false;

        addRequirements(s_Swerve);   
    }

    @Override
    public void initialize() {
        isRunning = true;

        targetPose = targetPoseSupplier.get();

        this.driveController = new ProfiledPIDController(DriveCommandConstants.kPXController, 0, 0, new TrapezoidProfile.Constraints(
            DriveCommandConstants.kMaxSpeedMetersPerSecond, DriveCommandConstants.kMaxAccelerationMetersPerSecondSquared
        ));

        s_Swerve.zeroGyro();
        s_Swerve.resetOdometry(new Pose2d());
        driveController.reset(new Translation2d().getDistance(targetPose.getTranslation()));
    }

    @Override
    public void execute() {
        Pose2d currentPose = s_Swerve.getPose();
        System.out.println("Drive: " + currentPose);
        
        /* reduce current distance (error) to 0 */
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double driveVelocityScalar = driveController.atGoal() ? 0.0 : 
            driveController.calculate(currentDistance, 0.0);

        Translation2d driveVelocity = new Translation2d(
            driveVelocityScalar, 
            currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle()
        );

        s_Swerve.drive(driveVelocity, 0, true, true);
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
        return isRunning && driveController.atGoal();
    }

    public boolean isRunning() {
        return isRunning;
    }
}