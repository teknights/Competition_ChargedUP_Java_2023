// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax; 
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import com.revrobotics.SparkMaxAbsoluteEncoder.*; 
import com.revrobotics.SparkMaxPIDController; 
import com.revrobotics.AbsoluteEncoder;



public class ArmDrive extends SubsystemBase {
  // Creates a new ArmDrive. 
   public static CANSparkMax ArmDriveMotor = new CANSparkMax(6, MotorType.kBrushed);
   public static AbsoluteEncoder m_enc = ArmDriveMotor.getAbsoluteEncoder(Type.kDutyCycle); 
SparkMaxPIDController m_pid = ArmDriveMotor.getPIDController(); 

public void Arm_Drive()
{
  double ArmSpeed;
  XboxController armController = new XboxController(1);
  double armrightydeadband = MathUtil.applyDeadband(armController.getRightY(), 0.3);
  m_pid.setFeedbackDevice(m_enc);
  ArmSpeed = armrightydeadband;
    ArmDriveMotor.set(-armrightydeadband);
    //Proportional
    double kP = 0.1; 
    //Integral
    double kI = 1e-4;
    //Derivitave
    double kD = 1; 
    //limiter on the I term
    double kIz = 0; 
    //feed forward
    double kFF = 0; 
    double kMaxOutput = 0.25; 
    double kMinOutput = -0.25;
}


   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
