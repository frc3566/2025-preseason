package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkBase.IdleMode;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class Swerve {
        public static final double stickDeadband = 0.1;

        public static final SPI.Port navXPort = SPI.Port.kMXP;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.25);
        public static final double wheelBase = Units.inchesToMeters(24.25);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0);
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int driveContinuousCurrentLimit = 35;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        // public static final double driveKP = 0.02531425;
        public static final double driveKP = 0.000;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.3797225;
        public static final double driveKV = 3.1;
        public static final double driveKA = 0.026;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeedMetersPerSecond = 4.5;
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = true;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(162.42);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(201.00);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(61.19);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(57.04);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static class Shooter {
        public static int leftMotorID = 10;
        public static int rightMotorID = 11;
    }

    public static class Intake {
        public static int intakeMotorID = 9;
    }

    public static class Climber {
        public static int leftClimberID = 12;
        public static int rightClimberID = 13;
    }

    public static class Vision {
        public static final String aprilTagCameraName = "Limelight1";

        public static final double cameraHeightMeters = 0;
        public static final double aprilTagHeightMeters = 0.6;
        public static final double cameraPitchRadians = Math.toRadians(0);

        public static final Transform3d robotToCamera = new Transform3d(
                new Translation3d(Units.inchesToMeters(8.75), Units.inchesToMeters(0.375), Units.inchesToMeters(12.75)),
                new Rotation3d(0, 0, 0));

        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo
                .loadAprilTagLayoutField();

        public static AprilTagFieldLayout aprilTagFieldLayout() throws IOException {
            return AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.kDefaultField.toString());
        }
    }

    /* LENGTH > WIDTH */
    public static class FieldConstants {
        public static final double length = Units.inchesToMeters(54 * 12 + 3.25);
        public static final double width = Units.inchesToMeters(26 * 12 + 3.5);
    }

    public static final class Trajectory {
        public static final TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.5 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2.25 * Math.PI;

        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 5;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}