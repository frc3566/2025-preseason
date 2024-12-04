package frc.robot.commands.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WithStatus;
import frc.robot.commands.swerve.pid.Drive;
import frc.robot.commands.swerve.pid.Spin;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AlignWithAprilTag extends SequentialCommandGroup implements WithStatus {
    private Pose2d targetPose = new Pose2d();
    private List<Command> commandsWithStatus;
    
    public AlignWithAprilTag(Swerve s_Swerve, Vision s_Vision) {
        commandsWithStatus = List.of(
            new SupplyAprilTagPose(s_Vision, new Pose2d(), (pose) -> targetPose = pose),
            new Drive(s_Swerve, () -> targetPose),
            new Spin(s_Swerve, () -> targetPose)
        );

        addCommands(
            commandsWithStatus.toArray(Command[]::new)
        );
    }

    @Override
    public boolean isRunning() {
        return commandsWithStatus.stream().map(command -> {
            if (!(command instanceof WithStatus)) { throw new Error("Command " + command.getName() + " does not implement WithStatus"); }
            return (WithStatus) command;
        }).anyMatch(WithStatus::isRunning);
    }
}