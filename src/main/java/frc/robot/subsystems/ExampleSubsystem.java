// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.util.function.DoubleSupplier;


import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.MotorSafety;




    
   
   


public class ExampleSubsystem extends SubsystemBase {

  Servo servo;
  DoublePublisher xPub;
  DoublePublisher yPub;

  private double x = 0.0;
  private double y = 0.0;

  private double x_add = 0.01;
  private double y_add = 0.05;

  private double ServoVal;

  AddressableLED myLED;
  AddressableLEDBuffer myLEDBuffer;
  private boolean isLightOn;

  BooleanPublisher X_Button;
  BooleanPublisher A_Button;
  DoublePublisher stickY;
  XboxController controller;

  NetworkTableInstance inst;
  NetworkTable table;

  CANSparkFlex motor;
  MotorSafety watchdog;

   
 //private DifferentialDrive m_robotDrive;
private final CANSparkMax leftMotorLeader;
private final CANSparkMax rightMotorLeader;
private final CANSparkMax leftMotorFollower;
private final CANSparkMax rightMotorFollower;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem()
  {
    leftMotorLeader = new CANSparkMax(10, MotorType.kBrushless);
    rightMotorLeader = new CANSparkMax(11,MotorType.kBrushless);
    leftMotorFollower = new CANSparkMax(12, MotorType.kBrushless);
    rightMotorFollower = new CANSparkMax(13, MotorType.kBrushless);
    //m_robotDrive = new DifferentialDrive(leftMotorLeader, rightMotorLeader);

    leftMotorFollower.follow(leftMotorLeader);
    rightMotorFollower.follow(rightMotorLeader);
  
    

    rightMotorLeader.setInverted(true);

    

    controller = new XboxController(0);

    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("frc695_test_table");
    servo = new Servo(0);
    ServoVal = 0.1;

    myLED = new AddressableLED(1);
    myLEDBuffer = new AddressableLEDBuffer(5);
    myLED.setLength(myLEDBuffer.getLength());
    myLED.setData(myLEDBuffer);
    myLED.start();
    isLightOn = false;
    
    xPub = table.getDoubleTopic("x").publish();
    xPub.set(0);


    yPub = table.getDoubleTopic("y").publish();
    yPub.set(0);

    X_Button = table.getBooleanTopic("x button").publish();
    X_Button.set(false);

    A_Button = table.getBooleanTopic("a button").publish();
    A_Button.set(false);

    stickY = table.getDoubleTopic("stick y axis").publish();

    motor = new CANSparkFlex(49, MotorType.kBrushless);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  

public Command TankDrive(double Right, double Left){
  return new FunctionalCommand(


  // ** INIT
  ()-> {},
 
  // ** EXECUTE
  ()-> {

    if(controller.getRightY()!= 0){
      rightMotorLeader.set(Right);
      
    }
    if(controller.getLeftY()!= 0){
      leftMotorLeader.set(Left);
      
    }



  },
 
  // ** ON INTERRUPTED
  interrupted -> {},
 
  // ** END CONDITION
  ()-> false,


  // ** REQUIREMENTS
  this);

  }




  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

//Turn on LED Red with B Button
// public Command LEDColors(){
//     return new FunctionalCommand(
//     //INIT
//     ()-> LEDSet(),
    
//     //EXEC
//     ()-> {},

//     //ON INTERR
//     interrupted-> LEDinterrupt(),

//     //END
//     ()-> endLED(),

    
//     this);
//   }
//   // private void LEDSet(){
//   //  AddressableLEDBuffer yourLEDBuffer = new AddressableLEDBuffer(5);
//   //     if(isLightOn == false){
//   //       for(var i= 0; i< myLEDBuffer.getLength(); i++){
//   //        yourLEDBuffer.setRGB(i,255, 0, 0);
//   //       }
//   //       isLightOn = true;
//   //     }else{
//   //        for(var i= 0; i< myLEDBuffer.getLength(); i++){
//   //        yourLEDBuffer.setRGB(i,0, 0, 0);
//   //       }
//   //       isLightOn = false;
//   //     }
//   // myLED.setData(yourLEDBuffer);
//   // myLED.start();
//   // }


//   private void LEDinterrupt(){
     
//   }
//   private boolean endLED(){
//     return (true);
//   }

public Command motorTurn(DoubleSupplier num) {
  return new FunctionalCommand(() -> {}, () -> {motor.set(num.getAsDouble());}, interrupted -> {},  () -> false, this);
}



  public Command servoTurn() {
    //DoubleSupplier RightY) {
    return new FunctionalCommand(

      // INIT
      ()-> {},

      // EXECUTE
      () -> { 
        if (controller.getRightY() != 0) {
          servo.set((controller.getRightY()+1)/2);
          if(controller.getRightY()>0){
            int LEDSpeedParameterG = 0;
            if((controller.getRightY()+1)/2 <0.2 && (controller.getRightY()+1)/2 >0 ){
              LEDSpeedParameterG = 1;
            }
            if((controller.getRightY()+1)/2 <0.4 && (controller.getRightY()+1)/2 >0.2 ){
              LEDSpeedParameterG = 2;
            }
            if((controller.getRightY()+1)/2 <0.6 && (controller.getRightY()+1)/2 >0.4 ){
              LEDSpeedParameterG = 3;
            }
            if((controller.getRightY()+1)/2 <0.8 && (controller.getRightY()+1)/2 >0.6 ){
              LEDSpeedParameterG = 4;
            }
            if((controller.getRightY()+1)/2 <=1 && (controller.getRightY()+1)/2 >0.8 ){
              LEDSpeedParameterG = 5;
            }
            AddressableLEDBuffer greenLEDBuffer = new AddressableLEDBuffer(5);
            for(var i= 0; i< LEDSpeedParameterG; i++){
                  greenLEDBuffer.setRGB(i,0, 255, 0);
               }
               myLED.setData(greenLEDBuffer);
               myLED.start();
          }
          if(controller.getRightY()<0){
            //Set speed parameter for LED with multitude of variables T^T
            int LEDSpeedParameter = 0;
            if((controller.getRightY()+1)/2 <0.2 && (controller.getRightY()+1)/2 >=0 ){
              LEDSpeedParameter = 5;
            }
            if((controller.getRightY()+1)/2 <0.4 && (controller.getRightY()+1)/2 >0.2 ){
              LEDSpeedParameter = 4;
            }
            if((controller.getRightY()+1)/2 <0.6 && (controller.getRightY()+1)/2 >0.4 ){
              LEDSpeedParameter = 3;
            }
            if((controller.getRightY()+1)/2 <0.8 && (controller.getRightY()+1)/2 >0.6 ){
              LEDSpeedParameter = 2;
            }
            if((controller.getRightY()+1)/2 <=1 && (controller.getRightY()+1)/2 >0.8 ){
              LEDSpeedParameter = 1;
            }

            AddressableLEDBuffer redLEDBuffer = new AddressableLEDBuffer(5);
            for(var i=0; i< LEDSpeedParameter; i++){
              redLEDBuffer.setRGB(i, 255,0,0);
            }
            myLED.setData(redLEDBuffer);
            myLED.start();
          }
        } 
        
        if (controller.getRightY() == 0) {
          servo.set(0.475);
          AddressableLEDBuffer noneLEDBuffer = new AddressableLEDBuffer(5);
          for(var i=0; i< myLEDBuffer.getLength(); i++){
              noneLEDBuffer.setRGB(i,0,0,0);
            }
            myLED.setData(noneLEDBuffer);
            myLED.start();
        }
      },//ServoExecute(RightY.getAsDouble());},

      // ON INTERRUPTED
      interrupted -> {},

      // END
      () -> false,

    this);

  }


  


private void interrupt(){
servo.set(0.475);
}

private boolean endCondition(){
  return (false);
}







  @Override
  public void periodic() {

    /* x += x_add;
    y += y_add;

    if (x > 50) {
      x_add *= -1;
    }

    if (x < 0) {
      x_add *= -1;
    }

    if (y > 50) {
      y_add *= -1;
    }

    if (y < 0) {
      y_add *= -1;
    }

    xPub.set(x);
    yPub.set(y);
    */

   /*
    //CONTROL SERVO WITH BUTTONS

   if (controller.getXButton() == true){
      X_Button.set(true);
      ServoVal = 0.1;
      servo.set(ServoVal);  
    }else{
      servo.set(0.475);
      if(controller.getAButton() == true){
      A_Button.set(true);
      ServoVal = 0.9;
      servo.set(ServoVal);
    }else{
      servo.set(0.475);
    }
    }
*/


//CONTROL SERVO WITH RIGHT JOYSTICK
 /*if (controller.getRightY() < 0){
      ServoVal = 0.1;
      servo.set(ServoVal);  
    }else{
      servo.set(0.475);
      if(controller.getRightY() > 0){
      ServoVal = 0.9;
      servo.set(ServoVal);
    }else{
      servo.set(0.475);
    }
    }
*/
    
    


  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public Command exampleMethodCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'exampleMethodCommand'");
  }
}


