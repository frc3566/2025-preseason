package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber {
    public CANSparkMax leftClimber;
    public CANSparkMax rightClimber;

    public Climber() {
        leftClimber = new CANSparkMax(Constants.Climber.leftClimberID, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Constants.Climber.rightClimberID, MotorType.kBrushless);
        leftClimber.setInverted(false);
        rightClimber.setInverted(false);
        leftClimber.setSmartCurrentLimit(40);
        rightClimber.setSmartCurrentLimit(40);
    }

    public void setPower(double power) {
        leftClimber.set(power);
        rightClimber.set(power);
    }

    public void setLeftPower(double power){
        leftClimber.set(power);
    }

    public void setRightPower(double power){
        rightClimber.set(power);
    }

    public void stop() {
        leftClimber.stopMotor();
        rightClimber.stopMotor();
    }
}
