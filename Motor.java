package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;


public class Motor extends SubsystemBase {
    // PID Variables set start
    public SparkFlex motor;
   // public RelativeEncoder encoder;
     PIDController pid;

    double kP = 0.005;
    double kI = 0.000001;
    double kD = 0.002;
    double kFF = 0.0001;

    int dir = 0;

    double kMinOutput = -1;
    double kMaxOutput = 1;

    double setPoint;
    // PID Variables set end
    // Swerve Drive Variables set start
    DoubleSupplier FWD;
    DoubleSupplier STR;
    DoubleSupplier RCW;
    TalonFX speedMotor;
    TalonFX directionMotor;
    CANcoder encoder;
    double w;
    double l;
    double hyp;
    

 

    // PID Controller 2024
    public Motor() {
    // motor = new SparkFlex(49, SparkLowLevel.MotorType.kBrushless);
    // encoder = motor.getEncoder();

    // motor.restoreFactoryDefaults();
    // encoder.setPosition(0);

    // pid = motor.getPIDController();
    // //PID Variables Construct start
    // pid.setP(kP);
    // pid.setD(kD);
    // pid.setI(kI);
    // pid.setFF(kFF);

    //pid.setOutputRange(kMinOutput, kMaxOutput);

    // SmartDashboard.putNumber("kP", kP);
    // SmartDashboard.putNumber("kD", kD);
    // setPoint = 2000;
    //PID Variables Construct end
    //Swerve Drive Construct Start

    speedMotor = new TalonFX(23);
    directionMotor = new TalonFX(22);

    encoder = new CANcoder(21);
    encoder.setPosition(0);
    
    

    pid = new PIDController(kP, kI, kD);
    pid.enableContinuousInput(-180.0, 180.0);

    w = 100/2;
    l = 100/2;
    hyp = Math.sqrt(Math.pow(w,2)+Math.pow(l,2));

    //Swerve Drive Construct end
    }

    // public Command pidcontrolCommand() {
    // return new FunctionalCommand(
    // () -> {
    // kP = SmartDashboard.getNumber("kP", kP);
    // // kD = SmartDashboard.getNumber("kD", kD);
    // // kI = SmartDashboard.getNumber("kI", kI);
    // // kFF = SmartDashboard.getNumber("kFF", kFF);
    // pid.setP(kP);
    // pid.setD(kD);
    // pid.setI(kI);
    // // pid.setFF(kFF);
    // },

    // () -> {

    // pid.setReference(setPoint, CANSparkFlex.ControlType.kVelocity);
    // SmartDashboard.putNumber("Velocity",encoder.getVelocity());
    // // SmartDashboard. put some numbers on here that tell ur velocity so u can
    // grpah ok good luck sigmalpha gamma
    // },

    // interrupted -> {
    // pid.setReference(0, CANSparkFlex.ControlType.kVelocity);
    // },

    // () -> false,

    // this);
    // }

    // Swerve Drive Commands
    // _____________________________________________________________________

    

    public double speedCalculate(DoubleSupplier FWD, DoubleSupplier STR) {
        return Math.sqrt(Math.pow(FWD.getAsDouble(), 2) + Math.pow(STR.getAsDouble(), 2));
    }

    // public double RCWFWD(double FWD, double STR, double RCW){
    //     double R = Math.sqrt(Math.pow(FWD,2)+Math.pow(STR,2));
    //     return RCW * (l/R);
    // }

    // public double RCWSTR(double FWD, double STR, double RCW){
    //     double R = Math.sqrt(Math.pow(FWD,2)+Math.pow(STR,2));
    //     return RCW * (w/R);
    // }

    // public double FWD1(double FWD, double RCWFWD){
    //     return FWD - RCWFWD;
    // }

    // public double STR1(double STR, double RCWSTR){
    //     return STR- RCWSTR;
    // }
    //(FWD1(FWD.getAsDouble(),RCWFWD(FWD.getAsDouble(),STR.getAsDouble(),RCW.getAsDouble()))),(STR1(STR.getAsDouble(),RCWSTR(FWD.getAsDouble(),STR.getAsDouble(),RCW.getAsDouble())))
     
    public double RCWSTR(DoubleSupplier STR, DoubleSupplier RCW){
        double RCWSTR = RCW.getAsDouble() * w/hyp;
        return STR.getAsDouble() - RCWSTR;
    }

    public double RCWFWD(DoubleSupplier FWD, DoubleSupplier RCW){
        double RCWFWD = RCW.getAsDouble() * l/hyp;
        return FWD.getAsDouble() + RCWFWD;
    }
    public double steeringAngle(DoubleSupplier FWD, DoubleSupplier STR, DoubleSupplier RCW){
        
        SmartDashboard.putNumber("tangent2", Math.atan2(RCWSTR(STR,RCW) ,RCWFWD(FWD,RCW))* 180/Math.PI);
        SmartDashboard.putNumber("tan1", Math.atan(RCWFWD(FWD,RCW)/RCWSTR(STR,RCW))*180/Math.PI);
        SmartDashboard.putNumber("RCW-FWD",RCWFWD(FWD,RCW) );
        SmartDashboard.putNumber("RCW-STR", RCWSTR(STR,RCW));
        return Math.atan2(RCWSTR(STR,RCW) ,RCWFWD(FWD,RCW))* 180/Math.PI;
     } 
     //RCWFWD(FWD,RCW)
     // RCWSTR(STR,RCW)
     

    public double angleCalculate(double FWD, double STR){
        return (Math.atan2(FWD, STR));
    }

    // public double setAngle(DoubleSupplier RCW, DoubleSupplier angle){
    // //     double currentAngle = encoder.getDirection();
    //         return 1;
    //  }

    public Command swerveDrive(DoubleSupplier FWD, DoubleSupplier STR, DoubleSupplier RCW) {
        return new FunctionalCommand(
                () -> {
                    
                },
                () -> {
                    double encoderPosition = encoder.getAbsolutePosition().getValueAsDouble()*360;
                    double position = speedMotor.getPosition().getValueAsDouble();
                    double dFWD = FWD.getAsDouble();
                    double dSTR = STR.getAsDouble();
                    double dRCW = RCW.getAsDouble();
                    double steeringAngle = steeringAngle(FWD,STR,RCW);
                    double whatIsGoingOnInMyMotor = pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW));
                    

                    SmartDashboard.putNumber("FWD", dFWD);
                    SmartDashboard.putNumber("STR", dSTR);
                    SmartDashboard.putNumber("RCW", dRCW);
                    SmartDashboard.putNumber("position", position);
                    SmartDashboard.putNumber("encoderPosition", encoderPosition);
                    SmartDashboard.putNumber("steeringAngle", steeringAngle);
                    SmartDashboard.putNumber("WHAT", whatIsGoingOnInMyMotor);

                    // if(( -0.1 < FWD.getAsDouble() && FWD.getAsDouble() < 0.1) && ( -0.1 < STR.getAsDouble() && STR.getAsDouble() < 0.1) && ( -0.1 < RCW.getAsDouble() && RCW.getAsDouble() < 0.1)){
                    //     speedMotor.set(0);
                    //     directionMotor.set(0);
                    // }

                    // if(-0.1 < speedCalculate(FWD, STR) && speedCalculate(FWD, STR) <0.1){
                    //     speedMotor.set(0);
                    // }else{
                    //     if(speedCalculate(FWD,STR) < 0){
                    //         speedMotor.set(speedCalculate(FWD, STR)*-1);
                    //     }else{
                    //         speedMotor.set(speedCalculate(FWD, STR));
                    //     }
                    // }

                    // if(-0.1 < pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW)) && pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW))<0.1){
                    //     directionMotor.set(0);
                    // }else{
                    //    directionMotor.set(pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW)));
                    // }
                   
                    // if(-0.1 < speedCalculate(FWD,STR) && speedCalculate(FWD, STR) < 0.1 && -0.1 < dRCW && dRCW < 0.1){
                    //     speedMotor.set(0);
                    //     directionMotor.set(0);
                    // }else{
                        
                        double speedCalculate = speedCalculate(FWD, STR);
                        double angle = angleCalculate(dFWD, dSTR);


                        SmartDashboard.putNumber("angle", angle);
                        SmartDashboard.putNumber("speedCalculate", speedCalculate);
                        //SmartDashboard.putNumber("steeringAngle", steeringAngle);
                        
                        
                        SmartDashboard.putNumber("steering Angle", steeringAngle(FWD,STR,RCW));
                        if(-0.1 < speedCalculate(FWD, STR) && speedCalculate(FWD,STR) < 0.1){
                            speedMotor.set(0);
                            directionMotor.set(0);
                        }else{
                        directionMotor.set(pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW)));
                        speedMotor.set(speedCalculate(FWD, STR));
                        }
                            //Previously inside the parameters:    steeringAngle(FWD,STR,RCW)
                            
                       // speedMotor.set(speedCalculate);
                        
                        //directionMotor.set(steeringAngle);
                    //}
                    
                //     if(-0.1 < pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW)) 
                // && pid.calculate(encoder.getAbsolutePosition().getValueAsDouble()*360, steeringAngle(FWD,STR,RCW))< 0.1){
                //         directionMotor.set(0);
                //     }else{
                        
                //     }
                    

                   
                },
                interrupted -> {
                },

                () -> false,

                this);
    }

}
