package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // public DigitalInput sensor;
    public CANSparkMax intakeMotor;
    public double rtrigger;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeMotor.setSmartCurrentLimit(60);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void takeIn(){
        intakeMotor.set(1);
    }

    public void eject(){
        intakeMotor.set(-0.5);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}