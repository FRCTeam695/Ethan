// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.pidcontrol;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */



public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final pidcontrol pidcontroller = new pidcontrol();

  // Replace with CommandPS4Controller or CommandJoystick if needed
 // private final CommandXboxController m_driverController =
      //new CommandXboxController(0);
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
  private final Joystick m_leftJoystick =
      new Joystick(0);
  


  //private final DoubleSupplier leftYAxis = () -> m_leftJoystick.getRawAxis(1);
  private final DoubleSupplier rightYAxis = () -> m_driverController.getRawAxis(1);
  





  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  //  Trigger bButton = m_driverController.b();
    //BUTTON^^^





  }



  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition);
        

       
       // m_driverController.b().onTrue(m_exampleSubsystem.LEDColors());
       // m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.servoTurn());
       // m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.motorTurn(leftYAxis));
       // m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.TankDrive(rightYAxis,leftYAxis));
      // pidcontroller.setDefaultCommand(pidcontroller.pidcontrolCommand());
       m_driverController.b().whileTrue(pidcontroller.pidcontrolCommand());
       // .onTrue(runOnce(() -> m_exampleSubsystem.PID(), m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}

// git add .
// git commit -m '__'
// git push 