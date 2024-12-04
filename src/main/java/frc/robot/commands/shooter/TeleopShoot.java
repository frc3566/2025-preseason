package frc.robot.commands.shooter;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopShoot extends Command {
    private Shooter s_Shooter;
    private DoubleSupplier left, right;

    public TeleopShoot(Shooter s_Shooter, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        addRequirements(s_Shooter);
        this.s_Shooter = s_Shooter;
        this.left = leftTrigger;
        this.right = rightTrigger;
    }
    
    public void execute() {
        s_Shooter.setPower(left.getAsDouble() - right.getAsDouble());
    }
}
