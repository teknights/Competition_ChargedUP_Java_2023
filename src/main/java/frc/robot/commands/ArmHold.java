// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmHold extends CommandBase {
  
    AbsoluteEncoder m_enc = ArmDrive.ArmDriveMotor.getAbsoluteEncoder(Type.kDutyCycle); 
 SparkMaxPIDController m_pid = ArmDrive.ArmDriveMotor.getPIDController(); 
 double defaultrotations = m_enc.getPosition();
 double UpdatedRefrence;
 //Proportional
 double kP = 0.1; 
 //Integral
 double kI = 1e-4;
 //Derivative
 double kD = 1; 
 //limiter on the I term
 double kIz = 0; 
 //feed forward
 double kFF = 0; 
 double kMaxOutput = 0.5; 
 double kMinOutput = -0.5;
 // set PID coefficients
 
  public ArmHold() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pid.setP(kP);
    m_pid.setI(kI);
    m_pid.setD(kD);
    m_pid.setIZone(kIz);
    m_pid.setFF(kFF);
    m_pid.setOutputRange(kMinOutput, kMaxOutput);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
 double currentrotations;
    double ArmSpeed;
  XboxController armController = new XboxController(1);
  double armrightydeadband = MathUtil.applyDeadband(armController.getRightY(), 0.3);
  m_pid.setFeedbackDevice(m_enc);
  ArmSpeed = armrightydeadband;
    ArmDrive.ArmDriveMotor.set(-armrightydeadband);
   
       
//this holds position of the arm when the A button is pressed
   if(armController.getAButton()) {
      UpdatedRefrence = m_enc.getPosition();
      m_pid.setReference(UpdatedRefrence, CANSparkMax.ControlType.kPosition);
      
    }
        else{m_pid.setReference(defaultrotations, CANSparkMax.ControlType.kPosition);}   
        //this sets the top limit of the arm 
   /*/ if(currentrotations >= (0.43)) {
      m_pid.setReference(0.42, CANSparkMax.ControlType.kPosition);
    }
    if(currentrotations <= (0.1)) {
      m_pid.setReference(0.11, CANSparkMax.ControlType.kPosition);
   
    }*/
    
  }
  @Override
    public void execute() {
         // display PID coefficients on SmartDashboard
         SmartDashboard.putNumber("P Gain", kP);
         SmartDashboard.putNumber("I Gain", kI);
         SmartDashboard.putNumber("D Gain", kD);
         SmartDashboard.putNumber("I Zone", kIz);
         SmartDashboard.putNumber("Feed Forward", kFF);
         SmartDashboard.putNumber("Max Output", kMaxOutput);
         SmartDashboard.putNumber("Min Output", kMinOutput);
         SmartDashboard.putNumber("Inital Rotations", 0);
 
         
 
         SmartDashboard.putNumber("SetPoint", defaultrotations);
         SmartDashboard.putNumber("ProcessVariable", m_enc.getPosition());
 
         /**
      * Encoder position is read from a RelativeEncoder object by calling the
      * GetPosition() method.
      * 
      * GetPosition() returns the position of the encoder in units of revolutions
      */
     SmartDashboard.putNumber("Encoder Position", m_enc.getPosition());
     System.out.println("Encoder Position" + m_enc.getPosition());
 
     /**
      * Encoder velocity is read from a RelativeEncoder object by calling the
      * GetVelocity() method.
      * 
      * GetVelocity() returns the velocity of the encoder in units of RPM
      */
     SmartDashboard.putNumber("Encoder Velocity", m_enc.getVelocity());
     System.out.println("Encoder Velocity" + m_enc.getVelocity());
    }
}
