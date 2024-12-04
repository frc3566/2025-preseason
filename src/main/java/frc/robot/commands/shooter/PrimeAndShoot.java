package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

/**
 * Command to shoot a piece. 
 * Allows shooter to accelerate for 1 second before the intake feeds the piece into the shooter.
 */
public class PrimeAndShoot extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;

    private final double targetSpeed;

    private Timer timer = new Timer();

    public PrimeAndShoot(Shooter s_Shooter, Intake s_Intake, double targetSpeed) {
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.targetSpeed = targetSpeed;
        addRequirements(s_Shooter, s_Intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        s_Shooter.setPower(targetSpeed);
        if (timer.get() >= 1) {
            s_Intake.setPower(0.9);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
        s_Intake.stop();
        timer.stop();
    }

    @Override 
    public boolean isFinished() {
        return timer.get() >= 1.3;
    }
}
