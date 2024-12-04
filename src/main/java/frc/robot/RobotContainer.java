package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.autos.*;
import frc.robot.commands.intake.IntakeAndHold;
import frc.robot.commands.intake.IntakeTimed;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.pid.*;
import frc.robot.commands.vision.AlignWithAprilTag;
import frc.robot.commands.shooter.PrimeAndShoot;
import frc.robot.commands.shooter.PrimeWhileThenShoot;
import frc.robot.commands.shooter.TeleopShoot;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driver2 = new Joystick(1);

    /* Joystick Axes */
    private final int leftThumbXID = XboxController.Axis.kLeftX.value;
    private final int leftThumbYID = XboxController.Axis.kLeftY.value;
    private final int rightThumbXID = XboxController.Axis.kRightX.value;

    private final int leftTriggerID = XboxController.Axis.kLeftTrigger.value;
    private final int rightTriggerID = XboxController.Axis.kRightTrigger.value;

    /* Driver Buttons */
    private final JoystickButton kX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton kY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton kA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton kB = new JoystickButton(driver, XboxController.Button.kB.value);

    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    private final POVButton DPadUp = new POVButton(driver, 0);
    private final POVButton DPadDown = new POVButton(driver, 180);
    private final POVButton DPadLeft = new POVButton(driver, 270);
    private final POVButton DPadRight = new POVButton(driver, 90);

    private final POVButton DPadUp2 = new POVButton(driver2, 0);
    private final POVButton DPadDown2 = new POVButton(driver2, 180);

    private final JoystickButton kX2 = new JoystickButton(driver2, XboxController.Button.kX.value);
    private final JoystickButton kY2 = new JoystickButton(driver2, XboxController.Button.kY.value);
    private final JoystickButton kA2 = new JoystickButton(driver2, XboxController.Button.kA.value);
    private final JoystickButton kB2 = new JoystickButton(driver2, XboxController.Button.kB.value);



    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Intake s_Intake = new Intake();
    private final Climber s_Climber = new Climber();
    private final Vision s_Vision;

    /* Auto Command */
    private final Command pathPlannerAuto;
    private Command alignCommand = new Spin(s_Swerve, () -> new Pose2d(0, 0, new Rotation2d(90)));
    private boolean alignCommandIsRunning = false;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        CommandScheduler.getInstance().onCommandInitialize(command -> System.out.println("Command initialized: " + command.getName()));
        CommandScheduler.getInstance().onCommandInterrupt(command -> System.out.println("Command interrupted: " + command.getName()));
        CommandScheduler.getInstance().onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

        s_Vision = new Vision();
        s_Vision.resetPose();
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(leftThumbYID), // translation axis
                () -> driver.getRawAxis(leftThumbXID), // strafe axis
                () -> -driver.getRawAxis(rightThumbXID),  // rotation axis
                () -> true // always field relative
            )
        );

        s_Shooter.setDefaultCommand(
            new TeleopShoot(
                s_Shooter, 
                () -> driver.getRawAxis(leftTriggerID),
                () -> driver.getRawAxis(rightTriggerID)
            )
        );

        /* By pausing init for a second before setting module offsets, we avoid a bug with inaccurate encoder readouts.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(4);
        s_Swerve.resetModulesToAbsolute();

        configureButtonBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        configurePathPlanner();
        pathPlannerAuto = new PathPlannerAuto("Preseason Test");
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@c
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        kX.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        kY.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        
        kB.whileTrue(new InstantCommand(() -> {
            if (!alignCommandIsRunning && s_Vision.getAprilTag().isPresent()) {
                s_Swerve.off();
                AlignWithAprilTag alignWithAprilTag = new AlignWithAprilTag(s_Swerve, s_Vision);
                alignCommand = alignWithAprilTag
                    .alongWith(new PrimeWhileThenShoot(s_Shooter, s_Intake, 1, () -> !alignWithAprilTag.isRunning()))
                    .finallyDo(() -> alignCommandIsRunning = false)
                    .withName("AlignWhilePriming");
                
                alignCommand.schedule();
                alignCommandIsRunning = true;
            }
        }).withName("InitiateAlignWithAprilTag").repeatedly());

        kA.onTrue(new InstantCommand(() -> {
            if (alignCommand != null) {
                alignCommand.cancel();
                alignCommandIsRunning = false;
            }
            alignCommand = null;
        }).withName("Cancel Align Command"));

        rightBumper.onTrue(new IntakeAndHold(s_Intake, s_Shooter, () -> rightBumper.getAsBoolean()));
        leftBumper.onTrue(new InstantCommand(() -> s_Intake.eject()));
        leftBumper.onFalse(new InstantCommand(() -> s_Intake.stop()));

        DPadUp.onTrue(new InstantCommand(() -> s_Climber.setPower(1.0)));
        DPadUp.onFalse(new InstantCommand(() -> s_Climber.setPower(0)));
        DPadDown.onTrue(new InstantCommand(() -> s_Climber.setPower(-0.8)));
        DPadDown.onFalse(new InstantCommand(() -> s_Climber.setPower(0)));

        DPadLeft.onTrue(new PrimeAndShoot(s_Shooter, s_Intake, 1.0));

        DPadUp2.onTrue(new InstantCommand(() -> s_Climber.setLeftPower(0.4)));
        DPadUp2.onFalse(new InstantCommand(() -> s_Climber.setLeftPower(0)));
        DPadDown2.onTrue(new InstantCommand(() -> s_Climber.setLeftPower(-0.4)));
        DPadDown2.onFalse(new InstantCommand(() -> s_Climber.setLeftPower(0)));

        kY2.onTrue(new InstantCommand(() -> s_Climber.setRightPower(0.4)));
        kY2.onFalse(new InstantCommand(() -> s_Climber.setRightPower(0)));
        kA2.onTrue(new InstantCommand(() -> s_Climber.setRightPower(-0.4)));
        kA2.onFalse(new InstantCommand(() -> s_Climber.setRightPower(0)));
    }

    /** 
     * Use this method to configure PathPlanner settings 
     * and expose commands to PathPlanner.
     */
    private void configurePathPlanner() {
        var translationPID = new PIDConstants(4.0, 0.0, 0.0);
        var rotationPID = new PIDConstants(6.0, 0.0, 0.0);
        var centerToFurthestModule = Constants.Swerve.wheelBase * Math.sqrt(2) / 2;

        var config = new HolonomicPathFollowerConfig(
            translationPID,
            rotationPID,
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
            centerToFurthestModule, 
            new ReplanningConfig() 
        );

        /* Paths should be flipped if we are on Red Alliance side */
        BooleanSupplier shouldFlipPath = () -> { return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red; };

        AutoBuilder.configureHolonomic(
            s_Swerve::getPose, 
            s_Swerve::resetOdometry, 
            s_Swerve::getRobotRelativeSpeeds, 
            s_Swerve::driveRobotRelative, 
            config,
            shouldFlipPath,
            s_Swerve
        );

        /* Register PathPlanner commands here */
        NamedCommands.registerCommand("PrimeAndShoot", new PrimeAndShoot(s_Shooter, s_Intake, 1.0));
        NamedCommands.registerCommand("Intake", new IntakeAndHold(s_Intake, s_Shooter, () -> true));
        NamedCommands.registerCommand("ReverseIntake", new IntakeTimed(s_Intake, () -> -0.1, 0.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return pathPlannerAuto;
    }
}