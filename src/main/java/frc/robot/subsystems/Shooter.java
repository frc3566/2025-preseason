package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public CANSparkMax left, right;

    public Shooter() {
        left = new CANSparkMax(Constants.Shooter.leftMotorID, MotorType.kBrushless);
        right = new CANSparkMax(Constants.Shooter.rightMotorID, MotorType.kBrushless);
        left.setInverted(false);
        right.setInverted(true);
        left.setSmartCurrentLimit(60);
        right.setSmartCurrentLimit(60);
    }

    /**
     * @param power The power to set both shooter motors to. 
     *      Value should be in the range [-1, 1] where 1 is full speed forward.
     *      Invert the motors in the constructor if behavior is not as described above.
     */
    public void setPower(double power) {
        left.set(power);
        right.set(power);
    }

    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }
    
}