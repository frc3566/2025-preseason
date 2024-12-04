package frc.robot.commands.intake;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTimed extends Command {
    private Intake s_Intake;

    private DoubleSupplier power;

    private Timer timer = new Timer();
    private double deadline;

    public IntakeTimed(Intake s_Intake, DoubleSupplier power, double seconds) {
        this.s_Intake = s_Intake;
        this.power = power;
        this.deadline = seconds;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        s_Intake.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stop();
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= deadline;
    }
}
