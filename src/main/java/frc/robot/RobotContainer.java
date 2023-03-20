// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import frc.robot.subsystems.ArmDrive;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //creates new armdrive
  ArmDrive m_ArmDrive = new ArmDrive();
  //Creates newIMU Sensor
  public static ADIS16470_IMU IMU = new ADIS16470_IMU();
  //Creates new limit switches
public DigitalInput VerticalLimitFoldedIn = new DigitalInput(Constants.VerticalLimitFoldedIn);
public DigitalInput VerticalLimitFoldedOut = new DigitalInput(Constants.VerticalLimitFoldedOut);
public DigitalInput NoTowerLean = new DigitalInput(Constants.NoTowerLean);
public DigitalInput FloorHit = new DigitalInput(Constants.FloorHit);
//this is for the encoder and PID controller
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //Driver Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
    Drivetrain drivetrain = new Drivetrain();
    XboxController drivController = new XboxController(OperatorConstants.kDriverControllerPort);
//Arm Controller
    public final static CommandXboxController m_armController =
    new CommandXboxController(OperatorConstants.kArmControllerPort);

    XboxController armController = new XboxController(OperatorConstants.kArmControllerPort);
    



    
    

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //this sends the Gyro Data to ShuffleBoard
    Shuffleboard.getTab("IMU Sensor").add("Gyro", IMU);
    ArmDrive.m_enc.setPositionConversionFactor(10000);
    SmartDashboard.putNumber("Encoder Position", ArmDrive.m_enc.getPosition());

    // Configure the trigger bindings
    configureBindings();
//Calls drivetrain when nothing is using the driving motors for example during auto this line is disregarded until drivetrain is unused again and sets drivtrain to roughly 70% power
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.drive(((-drivController.getLeftY()*drivController.getLeftY())/1.2), ((drivController.getRightX()*drivController.getRightX())/1.2)), drivetrain));
    m_ArmDrive.setDefaultCommand(new RunCommand(() -> m_ArmDrive.Arm_Drive(),m_ArmDrive)); 
    //ArmDrive.setDefaultCommand(new RunCommand(() -> new ArmHold()));
     //RB button Arm Controller pushes arm forward
    // m_armController.rightBumper().onTrue(new ArmKickDeploy());
     //LB Button Arm Controller retracts arm
   // m_armController.leftBumper().onTrue(new ArmKickRetract());
    //new ArmHold();

    //Limits arm deploy by encoder
   /*if(ArmDrive.currentrotations > (0.2)) {
      //lB button Arm Controller pushes arm forward
    m_armController.leftBumper().onTrue(new ArmKickDeploy());
    //RB Button Arm Controller retracts arm
   m_armController.rightBumper().onTrue(new ArmKickRetract());   } */
   
      

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
     CameraServer.startAutomaticCapture("0", 0);
    }
}
