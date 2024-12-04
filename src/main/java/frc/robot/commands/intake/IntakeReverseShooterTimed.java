package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

public class IntakeReverseShooterTimed extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;

    private DoubleSupplier shooterSpeed; 
    private DoubleSupplier intakeSpeed;

    private Timer timer = new Timer();
    private double deadline;

    public IntakeReverseShooterTimed(Intake s_Intake, Shooter s_Shooter, DoubleSupplier intakeSpeed, DoubleSupplier shooterSpeed, double seconds) {
        this.s_Intake = s_Intake;
        this.s_Shooter = s_Shooter;
        this.intakeSpeed = intakeSpeed;
        this.shooterSpeed = shooterSpeed;
        this.deadline = seconds;
        addRequirements(s_Intake, s_Shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        s_Shooter.setPower(shooterSpeed.getAsDouble());
        s_Intake.setPower(intakeSpeed.getAsDouble());
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
        s_Intake.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= deadline;
    }
}
