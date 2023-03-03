// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmKickDeploy;
import frc.robot.commands.ArmKickRetract;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //Driver Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    Drivetrain drivetrain = new Drivetrain();
    XboxController drivController = new XboxController(OperatorConstants.kDriverControllerPort);
//Arm Controller
    private final CommandXboxController m_armController =
    new CommandXboxController(OperatorConstants.kArmControllerPort);

    XboxController armController = new XboxController(OperatorConstants.kArmControllerPort);
    
    

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
//Calls drivetrain when nothing is using the driving motors for example during auto this line is disregarded until drivetrain is unused again
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(-drivController.getLeftY()*drivController.getLeftY(), drivController.getRightX()*drivController.getRightX()), drivetrain));
     //lB button Arm Controller pushes arm forward
    m_armController.leftBumper().onTrue(new ArmKickDeploy());
     //RB Button Arm Controller retracts arm
    m_armController.rightBumper().onTrue(new ArmKickRetract());

    //LT button opens claw
    m_armController.leftTrigger().onTrue(new Left_Claw_Open());
    m_armController.leftTrigger().onTrue(new Right_Claw_Open());
    //RT button closes Claw
    m_armController.rightTrigger().onTrue(new Left_Claw_Close());
    m_armController.rightTrigger().onTrue(new Right_Claw_Close());
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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
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

  /** This function is called periodically whilst in simulation. 
   * @param robot TODO*/
  public void simulationPeriodicInit(Robot robot) {
     robot.extracted();
    CameraServer.startAutomaticCapture("0", 0);}
}
