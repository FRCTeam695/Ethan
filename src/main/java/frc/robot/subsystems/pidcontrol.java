package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pidcontrol extends SubsystemBase{
    
    public CANSparkFlex motor;
    public RelativeEncoder encoder;
    public SparkPIDController pid;

    double kP = 0.00005;
    double kI = 0.000001;
    double kD = 0.002;
    double kFF = 0.0001;

    int dir = 0;

    double kMinOutput = -1;
    double kMaxOutput = 1;

    double setPoint;

    public pidcontrol() {
        motor = new CANSparkFlex(49, CANSparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();

        motor.restoreFactoryDefaults();
        encoder.setPosition(0);

        pid = motor.getPIDController();

        pid.setP(kP);
        pid.setD(kD);
        pid.setI(kI);
        pid.setFF(kFF);

        pid.setOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kD", kD);
        setPoint = 2000;
    }

    public Command pidcontrolCommand() {
        return new FunctionalCommand(
            () -> {
                kP = SmartDashboard.getNumber("kP", kP);
                // kD = SmartDashboard.getNumber("kD", kD);
                // kI = SmartDashboard.getNumber("kI", kI);
                // kFF = SmartDashboard.getNumber("kFF", kFF);
                pid.setP(kP);
                 pid.setD(kD);
                 pid.setI(kI);
                // pid.setFF(kFF);
            },

            () -> {

                pid.setReference(setPoint, CANSparkFlex.ControlType.kVelocity);
                SmartDashboard.putNumber("Velocity",encoder.getVelocity());
                // SmartDashboard. put some  numbers on here that tell ur velocity so u can grpah ok good luck sigmalpha gamma
            },

            interrupted -> {
                pid.setReference(0, CANSparkFlex.ControlType.kVelocity);
            },

            () -> false,

        this);
    }

    
}
