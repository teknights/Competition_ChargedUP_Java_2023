// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.RobotContainer.*;
import frc.robot.commands.ArmKickDeploy;
import frc.robot.commands.ArmKickRetract;
import frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import com.revrobotics.SparkMaxAbsoluteEncoder.*; 
import com.revrobotics.SparkMaxPIDController; 
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ControlType.*;
//remember * is a wildcard that imports everything to do with what it's calling just not one specific things but everything to do with what its calling




public class ArmDrive extends SubsystemBase implements PIDOutput {
  // Creates a new ArmDrive. 
   public static CANSparkMax ArmDriveMotor = new CANSparkMax(6, MotorType.kBrushed);
   public static AbsoluteEncoder m_enc = ArmDriveMotor.getAbsoluteEncoder(Type.kDutyCycle); 
public SparkMaxPIDController m_pid = ArmDriveMotor.getPIDController();

public static double currentrotations;
public static double angle = 45;

private final double Kp = 0.3;
private final double Ki = 0;
private final double Kd = 0;

public DigitalInput VerticalLimitFoldedIn = new DigitalInput(Constants.VerticalLimitFoldedIn);
public DigitalInput VerticalLimitFoldedOut = new DigitalInput(Constants.VerticalLimitFoldedOut);
public DigitalInput NoTowerLean = new DigitalInput(Constants.NoTowerLean);
public DigitalInput FloorHit = new DigitalInput(Constants.FloorHit);


public void Arm_Drive()
{
 // CANSparkMax ArmDriveMotor = new CANSparkMax(6, MotorType.kBrushed);
   AbsoluteEncoder m_enc = ArmDriveMotor.getAbsoluteEncoder(Type.kDutyCycle); 
 SparkMaxPIDController m_pid = ArmDriveMotor.getPIDController();
  XboxController armController = new XboxController(1);
  double armrightydeadband = (MathUtil.applyDeadband(armController.getRightY(), 0.3) / 2.15);
    //will need to remove line 69 to implement limit switches when they are ready
  //ArmDriveMotor.set(-armrightydeadband);

  
    if (NoTowerLean.get()) {
        // this makes sure the tower is up high enough to kick forward if True than it will not allow the arm to be kicked
        if(RobotContainer.m_armController.getHID().getRightBumper() || RobotContainer.m_armController.getHID().getLeftBumper()) {System.out.println("TOWER CANNOT BE DEPLOYED OR RETRACTED. PLEASE RAISE THE HEIGHT OF THE ARM");}
    } else {
        //this allows for the arm to be deployed and retracted
        //RB button Arm Controller pushes arm forward
     RobotContainer.m_armController.rightBumper().onTrue(new ArmKickDeploy());
     //LB Button Arm Controller retracts arm
     RobotContainer.m_armController.leftBumper().onTrue(new ArmKickRetract());
    }
 
    if (FloorHit.get()) {
      // This stops the motor from any negative value when/if it hits the floor
      if (armrightydeadband < 0){
        ArmDriveMotor.set(0);
        System.out.println("YOU HAVE HIT THE FLOOR!!! ARM MOTOR HAS BEEN PREVENTED FROM GOING ANY LOWER AND DRIVE HAS BEEN DISABLED!!! PLEASE BRING ARM UP TO REGAIN FULL CONTROL!");
        Drivetrain.arcadeDrive(0,0); //This should set drive motors to 0 to prevent movement
        Drivetrain.left_front.set(0);
        Drivetrain.left_back.set(0);
        Drivetrain.right_front.set(0);
        Drivetrain.right_back.set(0);
        
      }
      else{
          ArmDriveMotor.set(-armrightydeadband); //this is negative because the joysticks are inverted from the motor
        }
    } 
    if(VerticalLimitFoldedIn.get()) {
        // We are going down but bottom limit is not tripped so go at commanded speed
        ArmDriveMotor.set(0);
        System.out.println("Maximum Height Reached!!! Motor will now fall until ot reaches a safe height!");
    }
      else{
        ArmDriveMotor.set(-armrightydeadband);
      }
      if(VerticalLimitFoldedOut.get()){
        ArmDriveMotor.set(0);
        System.out.println("Maximum Height Reached!!! Motor will now fall until ot reaches a safe height!");
      }
        else {
          ArmDriveMotor.set(-armrightydeadband);
        }
        
        m_pid.setFeedbackDevice(m_enc);

    }
    




    /*/
    m_pid.enableContinuousInput(0, 90); 
    MathUtil.clamp(m_pid.calculate(m_enc.getPosition(), angle), -0.5, 0.5);
    m_pid.setTolerance(2);
    
    */

  

public void rotateDegrees(double angle){
  /*
  m_pid.reset();
  m_pid.setPID(Kp, Ki, Kd);
  m_pid.setSetpoint(angle);
  m_pid.calculate(angle);
  */
}

public void set(double rightvalue) {
ArmDriveMotor.set(rightvalue);



}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_enc.setPositionConversionFactor(10000);
    SmartDashboard.putNumber("Encoder Position", m_enc.getPosition());

  }





  @Override
  public void pidWrite(double Output) 
  {
    set(Output);
    // TODO Auto-generated method stub

    
  }





  
  
}
